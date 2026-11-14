"""
Project Eagle-Eye: Real-Time OpenCV HUD Dashboard & Video Recorder
Renders artificial horizon, tracking vectors, Kalman predictions, GPS bearing/distance,
battery voltage status, and active flight mode.
"""

import math
from typing import Any

import cv2
import numpy as np
from lora_link import (
    MODE_FAILSAFE_BLIND,
    MODE_GPS_PURSUIT,
    MODE_KALMAN_COAST,
    MODE_MANUAL_FPV,
    MODE_RETURN_TO_CAR,
    MODE_VISION_TRACK,
    DownlinkPacket,
    UplinkPacket,
)
from pursuit_controller import GPSMath


class HUDDashboard:
    def __init__(self, output_video_path: str | None = None, fps: int = 30):
        self.output_video_path = output_video_path
        self.fps = fps
        self.video_writer: cv2.VideoWriter | None = None

        # Mode names and colors (BGR)
        self.MODE_LABELS = {
            MODE_MANUAL_FPV: ("MANUAL FPV OVERRIDE", (0, 0, 255)),  # Red
            MODE_VISION_TRACK: ("VISION AUTONOMOUS TRACK", (0, 255, 0)),  # Green
            MODE_KALMAN_COAST: ("KALMAN COAST (ORBIT DECAY)", (0, 255, 255)),  # Yellow
            MODE_GPS_PURSUIT: ("GPS PURSUIT FALLBACK", (255, 128, 0)),  # Orange
            MODE_RETURN_TO_CAR: ("RETURN TO CAR (FAILSAFE)", (255, 0, 255)),  # Magenta
            MODE_FAILSAFE_BLIND: ("FAILSAFE BLIND LOITER", (0, 0, 255)),  # Red
        }

    def render(
        self,
        frame: np.ndarray,
        tracking_info: dict[str, Any],
        uplink: UplinkPacket,
        downlink: DownlinkPacket,
        car_lat: float,
        car_lon: float,
        manual_override_active: bool,
    ) -> np.ndarray:
        """
        Draws comprehensive HUD graphics onto the video frame
        """
        hud_frame = frame.copy()
        h, w = hud_frame.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Video Recording Initialization
        if self.output_video_path and self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.output_video_path, fourcc, self.fps, (w, h))

        # 2. Draw Target Bounding Box & Center Reticle
        if tracking_info.get("detected", False) and tracking_info.get("bbox") is not None:
            bx1, by1, bx2, by2 = tracking_info["bbox"]
            cv2.rectangle(hud_frame, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
            tcx, tcy = int((bx1 + bx2) / 2), int((by1 + by2) / 2)
            cv2.circle(hud_frame, (tcx, tcy), 5, (0, 255, 0), -1)

            # Draw vector from screen center to target center
            cv2.line(hud_frame, (cx, cy), (tcx, tcy), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(
                hud_frame,
                "TARGET LOCK: CAR",
                (bx1, max(20, by1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        # 3. Draw Kalman Prediction Bounding Box & Velocity Vector
        if tracking_info.get("kalman_pred_bbox") is not None:
            px1, py1, px2, py2 = tracking_info["kalman_pred_bbox"]
            # Clamp to frame
            px1, py1 = max(0, px1), max(0, py1)
            px2, py2 = min(w - 1, px2), min(h - 1, py2)
            if px2 > px1 and py2 > py1:
                cv2.rectangle(hud_frame, (px1, py1), (px2, py2), (255, 200, 0), 1, cv2.LINE_4)
                k_cx, k_cy = int((px1 + px2) / 2), int((py1 + py2) / 2)
                vx, vy = tracking_info.get("kalman_velocity", (0.0, 0.0))
                # Scale velocity vector for visual display (0.3s projection)
                end_x = int(k_cx + vx * 0.3)
                end_y = int(k_cy + vy * 0.3)
                cv2.arrowedLine(
                    hud_frame,
                    (k_cx, k_cy),
                    (end_x, end_y),
                    (0, 255, 255),
                    2,
                    tipLength=0.3,
                )
                cv2.putText(
                    hud_frame,
                    "KALMAN PREDICT",
                    (px1, min(h - 10, py2 + 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 200, 0),
                    1,
                    cv2.LINE_AA,
                )

        # 4. Center Crosshairs (Flight Boresight)
        cv2.line(hud_frame, (cx - 20, cy), (cx - 6, cy), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(hud_frame, (cx + 6, cy), (cx + 20, cy), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(hud_frame, (cx, cy - 20), (cx, cy - 6), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(hud_frame, (cx, cy + 6), (cx, cy + 20), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(hud_frame, (cx, cy), 3, (255, 255, 255), -1)

        # 5. Artificial Horizon Ladder (Roll & Pitch indication)
        roll_rad = math.radians(uplink.roll_deg)
        pitch_px = int(uplink.pitch_deg * 4.0)  # 4 pixels per degree
        horizon_len = 60
        dx = int(horizon_len * math.cos(roll_rad))
        dy = int(horizon_len * math.sin(roll_rad))
        cv2.line(
            hud_frame,
            (cx - dx, cy + pitch_px - dy),
            (cx + dx, cy + pitch_px + dy),
            (0, 220, 220),
            2,
            cv2.LINE_AA,
        )

        # 6. Mode & Status Bar (Top Header)
        mode_id = uplink.mode
        mode_text, mode_color = self.MODE_LABELS.get(mode_id, ("UNKNOWN MODE", (255, 255, 255)))
        if manual_override_active:
            mode_text = "[OVERRIDE] " + mode_text

        # Semi-transparent top ribbon
        overlay = hud_frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 42), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.6, hud_frame, 0.4, 0, hud_frame)

        cv2.putText(
            hud_frame,
            f"MODE: {mode_text}",
            (15, 28),
            cv2.FONT_HERSHEY_DUPLEX,
            0.65,
            mode_color,
            2,
            cv2.LINE_AA,
        )

        # 7. Telemetry Dashboard (Right Column)
        dist_to_car = 0.0
        bearing_to_car = 0.0
        if car_lat != 0.0 and downlink.plane_lat != 0.0:
            dist_to_car = GPSMath.haversine_distance(downlink.plane_lat, downlink.plane_lon, car_lat, car_lon)
            bearing_to_car = GPSMath.bearing_degrees(downlink.plane_lat, downlink.plane_lon, car_lat, car_lon)

        # Battery warning indicator
        batt_color = (
            (0, 255, 0) if downlink.batt_v >= 10.5 else ((0, 255, 255) if downlink.batt_v >= 9.6 else (0, 0, 255))
        )

        telem_lines = [
            f"SPEED: {downlink.speed_kmh:.1f} km/h",
            f"ALT:   {downlink.alt_m:.0f} m",
            f"HDG:   {downlink.heading_deg:.0f} deg",
            f"CAR DIST: {dist_to_car:.1f} m",
            f"CAR BRG:  {bearing_to_car:.0f} deg",
            f"BATT:  {downlink.batt_v:.1f} V (3S)",
        ]

        # Telemetry panel background
        panel_x = w - 210
        cv2.rectangle(hud_frame, (panel_x - 5, 50), (w - 10, 215), (10, 10, 10), -1)
        cv2.rectangle(hud_frame, (panel_x - 5, 50), (w - 10, 215), (80, 80, 80), 1)

        for i, line in enumerate(telem_lines):
            c = batt_color if "BATT" in line else (240, 240, 240)
            cv2.putText(
                hud_frame,
                line,
                (panel_x + 5, 75 + i * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                c,
                1,
                cv2.LINE_AA,
            )

        # 8. Commanded Flight Actuations (Bottom Bar)
        bot_y = h - 15
        ctrl_str = (
            f"CMD: Roll={uplink.roll_deg:+.1f} deg | Pitch={uplink.pitch_deg:+.1f} deg | Thr={uplink.throttle_pct}%"
        )
        cv2.putText(
            hud_frame,
            ctrl_str,
            (15, bot_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Write frame to video file if recording enabled
        if self.video_writer is not None:
            self.video_writer.write(hud_frame)

        return hud_frame

    def close(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
