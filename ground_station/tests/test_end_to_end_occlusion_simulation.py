import pytest
import numpy as np
import cv2
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from video_tracker import AerialVideoTracker
from pursuit_controller import PursuitController
from lora_link import (
    MODE_VISION_TRACK, MODE_KALMAN_COAST, MODE_GPS_PURSUIT, DownlinkPacket
)


def create_synthetic_frame(car_visible: bool, car_x: int, car_y: int, car_w: int = 80, car_h: int = 40) -> np.ndarray:
    """
    Creates a 640x480 frame with road background and optional visible car box
    """
    frame = np.full((480, 640, 3), 50, dtype=np.uint8)
    # Draw gray road
    cv2.rectangle(frame, (0, 150), (640, 330), (80, 80, 80), -1)

    if car_visible:
        # Draw distinctive car rectangle (bright blue with yellow roof)
        x1, y1 = car_x - car_w // 2, car_y - car_h // 2
        x2, y2 = car_x + car_w // 2, car_y + car_h // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (220, 100, 20), -1)
        cv2.rectangle(frame, (x1 + 10, y1 + 5), (x2 - 10, y2 - 5), (0, 220, 220), -1)
    else:
        # Draw dark occlusion (tunnel / dense coastal trees)
        cv2.rectangle(frame, (200, 140), (440, 340), (20, 20, 20), -1)
        cv2.putText(frame, "TUNNEL OCCLUSION", (220, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return frame


def test_end_to_end_occlusion_and_recovery_pipeline():
    # Initialize tracker with CSRT fallback enabled for guaranteed test determinism
    tracker = AerialVideoTracker(use_yolo=False)
    pursuit = PursuitController(target_box_area=3200.0)

    car_lat = 15.4909
    car_lon = 73.8278
    plane_telem = DownlinkPacket(plane_lat=15.4915, plane_lon=73.8285, alt_m=50.0, speed_kmh=45.0, heading_deg=90.0)

    dt = 0.04 # 25 Hz (40 ms per frame)
    car_x = 320
    car_y = 240

    # Phase 1: Initialize tracking on visible car (10 frames)
    init_frame = create_synthetic_frame(car_visible=True, car_x=car_x, car_y=car_y)
    tracker.init_manual_roi(init_frame, (car_x - 40, car_y - 20, 80, 40))

    state_history = []

    for f in range(15):
        car_x += 2 # Moving right
        frame = create_synthetic_frame(car_visible=True, car_x=car_x, car_y=car_y)
        t_info = tracker.process_frame(frame, dt)
        uplink = pursuit.update(t_info, plane_telem, car_lat, car_lon, dt)
        state_history.append(uplink.mode)

    # Must be in VISION_TRACK during clear visibility
    assert state_history[-1] == MODE_VISION_TRACK, f"Expected VISION_TRACK, got {state_history[-1]}"

    # Phase 2: Car enters tunnel occlusion (target disappears for 1.0s = 25 frames)
    # Controller must enter KALMAN_COAST (0 - 2.5s)
    coast_states = []
    for f in range(25):
        frame = create_synthetic_frame(car_visible=False, car_x=car_x, car_y=car_y)
        t_info = tracker.process_frame(frame, dt)
        # Simulate time progression for test
        t_info["time_since_detect"] = (f + 1) * dt
        uplink = pursuit.update(t_info, plane_telem, car_lat, car_lon, dt)
        coast_states.append(uplink.mode)

    # Must transition to KALMAN_COAST during short loss
    assert MODE_KALMAN_COAST in coast_states
    assert coast_states[-1] == MODE_KALMAN_COAST

    # Phase 3: Occlusion persists > 2.5 seconds (another 45 frames -> total 2.8s)
    # Controller must transition to GPS_PURSUIT
    gps_pursuit_states = []
    for f in range(45):
        frame = create_synthetic_frame(car_visible=False, car_x=car_x, car_y=car_y)
        t_info = tracker.process_frame(frame, dt)
        t_info["time_since_detect"] = 1.0 + (f + 1) * dt
        uplink = pursuit.update(t_info, plane_telem, car_lat, car_lon, dt)
        gps_pursuit_states.append(uplink.mode)

    assert gps_pursuit_states[-1] == MODE_GPS_PURSUIT, f"Expected GPS_PURSUIT after >2.5s, got {gps_pursuit_states[-1]}"

    # Phase 4: Target emerges from occlusion!
    # Re-acquisition requires 10 consecutive frames before returning to VISION_TRACK
    # Frames 1-9: Still in GPS_PURSUIT counting re-lock frames
    relock_states = []
    for f in range(9):
        frame = create_synthetic_frame(car_visible=True, car_x=car_x, car_y=car_y)
        # Re-detect target
        t_info = {
            "detected": True,
            "bbox": (car_x - 40, car_y - 20, car_x + 40, car_y + 20),
            "area": 3200.0,
            "error_x_norm": 0.1,
            "error_y_norm": 0.0,
            "time_since_detect": 0.0
        }
        uplink = pursuit.update(t_info, plane_telem, car_lat, car_lon, dt)
        relock_states.append(uplink.mode)

    # At frame 9 (less than 10 consecutive), still in GPS_PURSUIT
    assert relock_states[-1] == MODE_GPS_PURSUIT

    # Frame 10: 10th consecutive frame -> Transitions back to VISION_TRACK!
    uplink_10th = pursuit.update(t_info, plane_telem, car_lat, car_lon, dt)
    assert uplink_10th.mode == MODE_VISION_TRACK, f"Expected VISION_TRACK after 10 frames, got {uplink_10th.mode}"
