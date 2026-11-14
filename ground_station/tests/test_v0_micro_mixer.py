import pytest


def compute_differential_thrust(
    throttle_pct: float,
    steer_input: float,
    gyro_yaw_rate: float,
    damping_gain: float = 0.15,
    max_diff: float = 45.0,
) -> tuple[float, float]:
    """
    Python mirror of C++ differential thrust mixer in firmware/v0_micro_fc/src/main.cpp
    """
    if throttle_pct <= 1.0:
        return 0.0, 0.0

    steer_effective = (steer_input / 100.0) * max_diff
    gyro_stabilization = -gyro_yaw_rate * damping_gain

    raw_left = throttle_pct + (steer_effective * 0.5) + gyro_stabilization
    raw_right = throttle_pct - (steer_effective * 0.5) - gyro_stabilization

    left = max(0.0, min(100.0, raw_left))
    right = max(0.0, min(100.0, raw_right))
    return left, right


def test_straight_level_flight():
    left, right = compute_differential_thrust(throttle_pct=60.0, steer_input=0.0, gyro_yaw_rate=0.0)
    assert left == 60.0
    assert right == 60.0


def test_right_turn_steering():
    # Steer right (+50%): Left motor speeds up, Right motor slows down
    left, right = compute_differential_thrust(throttle_pct=50.0, steer_input=50.0, gyro_yaw_rate=0.0)
    assert left > 50.0
    assert right < 50.0
    assert pytest.approx(left - right, abs=0.1) == 22.5  # 50% of 45% = 22.5% split


def test_left_turn_steering():
    # Steer left (-50%): Right motor speeds up, Left motor slows down
    left, right = compute_differential_thrust(throttle_pct=50.0, steer_input=-50.0, gyro_yaw_rate=0.0)
    assert left < 50.0
    assert right > 50.0


def test_gyro_yaw_damping():
    # Sudden uncommanded right yaw (+20 deg/s disturbance) with steer=0:
    # Mixer should automatically counter-steer (Right motor speeds up to counteract)
    left, right = compute_differential_thrust(throttle_pct=60.0, steer_input=0.0, gyro_yaw_rate=20.0)
    assert right > left


def test_zero_throttle_cutoff():
    # Even if full steering is input, motors must remain off when throttle is 0%
    left, right = compute_differential_thrust(throttle_pct=0.0, steer_input=100.0, gyro_yaw_rate=10.0)
    assert left == 0.0
    assert right == 0.0


def test_saturation_clamping():
    # 100% throttle with extreme inputs must not exceed 100.0
    left, right = compute_differential_thrust(throttle_pct=100.0, steer_input=100.0, gyro_yaw_rate=-50.0)
    assert left <= 100.0
    assert right >= 0.0
