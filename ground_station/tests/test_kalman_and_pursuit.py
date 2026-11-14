import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from kalman_tracker import ConstantVelocityKalmanTracker
from lora_link import MODE_VISION_TRACK, DownlinkPacket
from pursuit_controller import PIDVisualServoing, PursuitController


def test_kalman_constant_velocity_tracking():
    # True target starts at cx=100, moving with vx=50 px/sec
    kf = ConstantVelocityKalmanTracker(init_cx=100.0, init_cy=200.0, init_area=5000.0)
    dt = 0.04  # 25 Hz update rate

    true_cx = 100.0
    vx = 50.0

    for step in range(30):
        true_cx += vx * dt
        kf.predict(dt)
        # Add noisy measurement
        noisy_meas = true_cx + np.random.normal(0, 1.0)
        kf.update(noisy_meas, 200.0, 5000.0)

    est_cx, est_cy = kf.position
    est_vx, _est_vy = kf.velocity

    assert pytest.approx(est_cx, abs=5.0) == true_cx
    assert pytest.approx(est_vx, abs=10.0) == vx
    assert pytest.approx(est_cy, abs=5.0) == 200.0


def test_kalman_prediction_during_occlusion():
    kf = ConstantVelocityKalmanTracker(init_cx=200.0, init_cy=150.0, init_area=6000.0)
    dt = 0.1

    # Feed steady velocity vx=40 px/s, vy=20 px/s for 15 steps
    for _ in range(15):
        kf.predict(dt)
        kf.update(kf.x[0] + 4.0, kf.x[1] + 2.0, 6000.0)

    pos_before_occlusion = kf.position

    # Target occluded: predict for 10 steps (1.0 second) without measurements
    for _ in range(10):
        kf.predict(dt)

    pos_after_occlusion = kf.position
    # Should have continued in same direction based on velocity estimate
    assert pos_after_occlusion[0] > pos_before_occlusion[0]
    assert pos_after_occlusion[1] > pos_before_occlusion[1]


def test_visual_servoing_pid_bounds():
    pid = PIDVisualServoing(kp=35.0, ki=0.5, kd=2.0, min_val=-35.0, max_val=35.0)

    # Large positive error (+1.0 full right) -> output clamped to max +35.0 deg
    out_pos = pid.update(1.0, dt=0.04)
    assert out_pos <= 35.0
    assert out_pos > 30.0

    # Large negative error (-1.0 full left) -> output clamped to min -35.0 deg
    out_neg = pid.update(-1.0, dt=0.04)
    assert out_neg >= -35.0
    assert out_neg < -30.0


def test_pursuit_controller_vision_servo_response():
    pursuit = PursuitController(target_box_area=10000.0)
    telem = DownlinkPacket(plane_lat=15.0, plane_lon=73.0, alt_m=50.0, speed_kmh=45.0, heading_deg=90.0)

    # Target off-center to the right (error_x_norm = +0.5)
    tracking_info = {
        "detected": True,
        "error_x_norm": 0.5,
        "error_y_norm": 0.0,
        "area": 10000.0,
        "time_since_detect": 0.0,
    }

    uplink = pursuit.update(tracking_info, telem, car_lat=15.0, car_lon=73.0, dt=0.04)
    assert uplink.mode == MODE_VISION_TRACK
    assert uplink.roll_deg > 0.0  # Must bank right into turn
    assert 35 <= uplink.throttle_pct <= 85
