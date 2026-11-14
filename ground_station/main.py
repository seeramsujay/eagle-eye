"""
Project Eagle-Eye: Autonomous Airborne Pursuit Laptop Ground Station
Coordinates:
- Video Capture (UVC Camera / Video File / Test Generator)
- Target Tracking (YOLOv8n + CSRT + Kalman)
- Visual Servoing & 3-State Pursuit Controller
- Instant Manual FPV Override (< 5ms)
- LoRa 433MHz Telemetry Link (921600 baud binary framing)
- Real-time OpenCV HUD Dashboard
"""

import argparse
import time

import cv2
import numpy as np
from hud_dashboard import HUDDashboard
from lora_link import DownlinkPacket, LoRaLink
from manual_override import ManualOverrideManager
from pursuit_controller import PursuitController
from video_tracker import AerialVideoTracker


def parse_args():
    parser = argparse.ArgumentParser(
        description="Project Eagle-Eye Laptop Ground Station"
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Video source (camera index, RTSP URL, or video path)",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Ground LoRa Bridge USB serial port (e.g. /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        default=True,
        help="Run in mock/simulation mode without hardware",
    )
    parser.add_argument(
        "--record", type=str, default=None, help="Path to save MP4 flight recording"
    )
    parser.add_argument(
        "--car-lat",
        type=float,
        default=15.4909,
        help="Simulated car GPS latitude (Goa coast)",
    )
    parser.add_argument(
        "--car-lon",
        type=float,
        default=73.8278,
        help="Simulated car GPS longitude (Goa coast)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    print("=" * 65)
    print(" PROJECT EAGLE-EYE: GROUND CONTROL & PURSUIT SYSTEM ")
    print("=" * 65)

    # 1. Initialize Video Source
    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(
            f"[WARN] Cannot open video source {src}. Switching to synthetic test feed."
        )
        cap = None

    # 2. Initialize Subsystems
    tracker = AerialVideoTracker(use_yolo=True, model_name="yolov8n.pt")
    pursuit = PursuitController(target_box_area=12000.0)
    manual_mgr = ManualOverrideManager()
    manual_mgr.start()

    lora = LoRaLink(port=args.port, mock_mode=args.mock)
    lora.connect()

    hud = HUDDashboard(output_video_path=args.record)

    last_time = time.time()
    synthetic_x = 320
    synthetic_y = 240
    synthetic_vx = 35.0  # pixels per second

    print("\nControls:")
    print("  SPACE: Instant Manual FPV Takeover")
    print("  '1':   Manual FPV Mode")
    print("  '2':   Autonomous Vision Track Mode")
    print("  '3':   Return to Car Mode")
    print("  'q':   Quit Ground Station\n")

    try:
        while True:
            current_time = time.time()
            dt = max(0.001, min(0.1, current_time - last_time))
            last_time = current_time

            # Read or generate frame
            if cap is not None and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
            else:
                # Generate synthetic test frame (640x480) with simulated moving car
                frame = np.full((480, 640, 3), 40, dtype=np.uint8)
                # Draw road
                cv2.rectangle(frame, (0, 180), (640, 300), (70, 70, 70), -1)
                # Move simulated car box
                synthetic_x += synthetic_vx * dt
                if synthetic_x > 560 or synthetic_x < 80:
                    synthetic_vx = -synthetic_vx
                car_w, car_h = 100, 50
                x1 = int(synthetic_x - car_w // 2)
                y1 = int(synthetic_y - car_h // 2)
                cv2.rectangle(
                    frame, (x1, y1), (x1 + car_w, y1 + car_h), (200, 140, 40), -1
                )
                cv2.putText(
                    frame,
                    "SIMULATED CAR",
                    (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                )

            # 1. Process Computer Vision Tracking
            tracking_info = tracker.process_frame(frame, dt)

            # 2. Get latest plane telemetry
            plane_telem = lora.last_downlink or DownlinkPacket(
                plane_lat=15.4912,
                plane_lon=73.8280,
                alt_m=42.0,
                speed_kmh=45.0,
                heading_deg=90.0,
            )

            # 3. Compute Autonomous Pursuit Commands
            auto_uplink = pursuit.update(
                tracking_info, plane_telem, args.car_lat, args.car_lon, dt
            )

            # 4. Check & Apply Instant Manual FPV Override (< 5 ms)
            final_uplink, is_override = manual_mgr.apply_override_if_active(auto_uplink)

            # 5. Transmit 16-Byte Uplink over LoRa Link
            lora.send_uplink(final_uplink)

            # 6. Render HUD Overlay
            hud_frame = hud.render(
                frame,
                tracking_info,
                final_uplink,
                plane_telem,
                args.car_lat,
                args.car_lon,
                is_override,
            )

            # 7. Display on Screen
            cv2.imshow("Project Eagle-Eye Ground Station", hud_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            else:
                manual_mgr.handle_key(key)

    finally:
        if cap:
            cap.release()
        hud.close()
        lora.disconnect()
        manual_mgr.stop()
        cv2.destroyAllWindows()
        print("[OK] Ground Station shutdown cleanly.")


if __name__ == "__main__":
    main()
