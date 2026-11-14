import pytest
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from manual_override import ManualOverrideManager
from lora_link import UplinkPacket, MODE_VISION_TRACK, MODE_MANUAL_FPV


def test_instant_override_trigger_latency():
    mgr = ManualOverrideManager()
    # Trigger takeover and measure execution latency
    latencies = []
    for _ in range(50):
        lat = mgr.trigger_takeover("SPACE_KEY")
        latencies.append(lat)
        mgr.release_to_autonomy()

    max_lat = max(latencies)
    avg_lat = sum(latencies) / len(latencies)

    # Verification: Must strictly preempt in < 5.0 milliseconds
    assert max_lat < 5.0, f"Max preemption latency {max_lat:.3f} ms exceeded 5.0 ms ceiling"
    assert avg_lat < 1.0, f"Avg preemption latency {avg_lat:.3f} ms was unexpectedly high"


def test_override_command_preemption():
    mgr = ManualOverrideManager()
    mgr.set_sticks(roll=0.5, pitch=-0.3, throttle_pct=80)

    auto_cmd = UplinkPacket(mode=MODE_VISION_TRACK, roll_deg=10.0, pitch_deg=2.0, throttle_pct=50)

    # When override is inactive: auto command passes through
    out_cmd, is_override = mgr.apply_override_if_active(auto_cmd)
    assert not is_override
    assert out_cmd.mode == MODE_VISION_TRACK
    assert out_cmd.roll_deg == 10.0

    # Trigger manual takeover
    mgr.trigger_takeover("USER_HOTKEY")

    # When override is active: stick commands preempt immediately
    out_cmd2, is_override2 = mgr.apply_override_if_active(auto_cmd)
    assert is_override2
    assert out_cmd2.mode == MODE_MANUAL_FPV
    assert pytest.approx(out_cmd2.roll_deg, abs=0.1) == 0.5 * 35.0 # +17.5 deg
    assert pytest.approx(out_cmd2.pitch_deg, abs=0.1) == -0.3 * 15.0 # -4.5 deg
    assert out_cmd2.throttle_pct == 80


def test_keyboard_hotkeys():
    mgr = ManualOverrideManager()
    assert not mgr.manual_override_active

    # Press SPACE
    mgr.handle_key(32)
    assert mgr.manual_override_active
    assert mgr.desired_mode == MODE_MANUAL_FPV

    # Press '2' (Re-engage vision autonomy)
    mgr.handle_key(ord('2'))
    assert not mgr.manual_override_active
    assert mgr.desired_mode == MODE_VISION_TRACK
