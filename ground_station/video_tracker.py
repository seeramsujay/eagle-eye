"""
Project Eagle-Eye: Computer Vision Aerial Target Tracker
Integrates YOLOv8n ('car' class) + OpenCV CSRT Fallback with Appearance Verification
+ Constant Velocity Kalman Filter
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple, Dict, Any
from kalman_tracker import ConstantVelocityKalmanTracker


class AerialVideoTracker:
    def __init__(self, use_yolo: bool = True, model_name: str = "yolov8n.pt", conf_thresh: float = 0.35):
        self.use_yolo = use_yolo
        self.conf_thresh = conf_thresh
        self.yolo_model = None
        self.csrt_tracker: Optional[cv2.Tracker] = None
        self.reference_hist: Optional[np.ndarray] = None
        self.kalman: Optional[ConstantVelocityKalmanTracker] = None

        self.tracking_active = False
        self.consecutive_detect_frames = 0
        self.consecutive_lost_frames = 0
        self.last_detection_timestamp = 0.0

        if self.use_yolo:
            try:
                from ultralytics import YOLO
                self.yolo_model = YOLO(model_name)
            except Exception as e:
                print(f"[AerialVideoTracker] Warning: YOLO init failed ({e}). Defaulting to CSRT tracker.")
                self.use_yolo = False

    def init_manual_roi(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]):
        """
        Initializes tracking via manual ROI (x, y, w, h)
        """
        x, y, w, h = bbox
        if w < 5 or h < 5:
            return

        cx = x + w / 2.0
        cy = y + h / 2.0
        area = float(w * h)

        self.kalman = ConstantVelocityKalmanTracker(cx, cy, area)
        try:
            self.csrt_tracker = cv2.TrackerCSRT_create()
            self.csrt_tracker.init(frame, (x, y, w, h))

            # Store reference appearance histogram in HSV color space
            crop = frame[max(0, y):min(frame.shape[0], y + h), max(0, x):min(frame.shape[1], x + w)]
            if crop.size > 0:
                hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
                cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
                self.reference_hist = hist
        except Exception:
            self.csrt_tracker = None
            self.reference_hist = None

        self.tracking_active = True
        self.consecutive_detect_frames = 1
        self.consecutive_lost_frames = 0
        self.last_detection_timestamp = time.time()

    def process_frame(self, frame: np.ndarray, dt: float) -> Dict[str, Any]:
        """
        Processes a single video frame and returns tracking metadata
        """
        h, w = frame.shape[:2]
        img_center_x = w / 2.0
        img_center_y = h / 2.0
        current_time = time.time()

        result = {
            "detected": False,
            "bbox": None, # (x1, y1, x2, y2)
            "center": None, # (cx, cy)
            "area": None,
            "error_x_px": 0.0,
            "error_y_px": 0.0,
            "error_x_norm": 0.0, # [-1.0, 1.0]
            "error_y_norm": 0.0, # [-1.0, 1.0]
            "kalman_pred_bbox": None,
            "kalman_velocity": (0.0, 0.0),
            "consecutive_detects": self.consecutive_detect_frames,
            "time_since_detect": current_time - self.last_detection_timestamp if self.last_detection_timestamp > 0 else 999.0
        }

        # 1. Kalman Prediction step
        if self.kalman is not None:
            self.kalman.predict(dt)
            result["kalman_pred_bbox"] = self.kalman.get_predicted_bbox()
            result["kalman_velocity"] = self.kalman.velocity

        matched_bbox = None

        # 2. YOLOv8 Detection (Class 2 = car in COCO)
        if self.use_yolo and self.yolo_model is not None:
            try:
                detections = self.yolo_model(frame, classes=[2], conf=self.conf_thresh, verbose=False)[0]
                best_iou = 0.0
                best_box = None

                for box in detections.boxes:
                    bx1, by1, bx2, by2 = box.xyxy[0].cpu().numpy()
                    if self.kalman is not None:
                        # Find box closest to Kalman predicted bounding box
                        pred_box = result["kalman_pred_bbox"]
                        iou = self._calculate_iou((bx1, by1, bx2, by2), pred_box)
                        if iou > best_iou:
                            best_iou = iou
                            best_box = (int(bx1), int(by1), int(bx2), int(by2))
                    else:
                        area = (bx2 - bx1) * (by2 - by1)
                        if best_box is None or area > best_iou:
                            best_iou = area
                            best_box = (int(bx1), int(by1), int(bx2), int(by2))

                if best_box is not None and (self.kalman is None or best_iou > 0.15):
                    matched_bbox = best_box
            except Exception:
                pass

        # 3. Fallback to OpenCV CSRT Tracker if YOLO missed
        if matched_bbox is None and self.csrt_tracker is not None:
            ok, csrt_box = self.csrt_tracker.update(frame)
            if ok:
                x, y, bw, bh = [int(v) for v in csrt_box]
                # Appearance verification: check if cropped patch still matches target
                crop_y1 = max(0, y)
                crop_y2 = min(h, y + bh)
                crop_x1 = max(0, x)
                crop_x2 = min(w, x + bw)
                crop = frame[crop_y1:crop_y2, crop_x1:crop_x2]

                appearance_valid = True
                if self.reference_hist is not None and crop.size > 0:
                    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
                    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
                    sim = cv2.compareHist(self.reference_hist, hist, cv2.HISTCMP_CORREL)
                    # If correlation drops below 0.35, target is occluded / lost
                    if sim < 0.35:
                        appearance_valid = False

                if appearance_valid:
                    matched_bbox = (x, y, x + bw, y + bh)

        # 4. Update Kalman Filter or handle Lost State
        if matched_bbox is not None:
            x1, y1, x2, y2 = matched_bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            area = float(max(10, (x2 - x1) * (y2 - y1)))

            if self.kalman is None:
                self.kalman = ConstantVelocityKalmanTracker(cx, cy, area)
            else:
                self.kalman.update(cx, cy, area)

            self.tracking_active = True
            self.consecutive_detect_frames += 1
            self.consecutive_lost_frames = 0
            self.last_detection_timestamp = current_time

            result["detected"] = True
            result["bbox"] = matched_bbox
            result["center"] = (cx, cy)
            result["area"] = area
            result["error_x_px"] = cx - img_center_x
            result["error_y_px"] = cy - img_center_y
            result["error_x_norm"] = result["error_x_px"] / (w / 2.0)
            result["error_y_norm"] = result["error_y_px"] / (h / 2.0)
            result["consecutive_detects"] = self.consecutive_detect_frames
            result["time_since_detect"] = 0.0
        else:
            self.consecutive_detect_frames = 0
            self.consecutive_lost_frames += 1

            if self.kalman is not None:
                # Use Kalman prediction as estimated center
                k_cx, k_cy = self.kalman.position
                result["center"] = (k_cx, k_cy)
                result["area"] = self.kalman.area
                result["error_x_px"] = k_cx - img_center_x
                result["error_y_px"] = k_cy - img_center_y
                result["error_x_norm"] = result["error_x_px"] / (w / 2.0)
                result["error_y_norm"] = result["error_y_px"] / (h / 2.0)

        return result

    @staticmethod
    def _calculate_iou(boxA: Tuple[float, float, float, float], boxB: Tuple[float, float, float, float]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        interArea = max(0.0, xB - xA) * max(0.0, yB - yA)
        boxAArea = max(1.0, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        boxBArea = max(1.0, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
        return interArea / float(boxAArea + boxBArea - interArea)
