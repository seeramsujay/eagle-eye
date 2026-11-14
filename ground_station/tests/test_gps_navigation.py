import pytest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pursuit_controller import GPSMath


def test_haversine_distance():
    # Mumbai Gateway of India to Marine Drive (~3.2 km)
    lat1, lon1 = 18.9220, 72.8347
    lat2, lon2 = 18.9438, 72.8232

    dist = GPSMath.haversine_distance(lat1, lon1, lat2, lon2)
    assert 2500.0 < dist < 3000.0 # ~3.0 - 3.2 km


def test_bearing_calculation():
    # Due North: lat increases, lon constant
    brg_north = GPSMath.bearing_degrees(15.0, 73.0, 16.0, 73.0)
    assert pytest.approx(brg_north, abs=0.5) == 0.0

    # Due East: lat constant, lon increases
    brg_east = GPSMath.bearing_degrees(15.0, 73.0, 15.0, 74.0)
    assert pytest.approx(brg_east, abs=0.5) == 90.0

    # Due South
    brg_south = GPSMath.bearing_degrees(16.0, 73.0, 15.0, 73.0)
    assert pytest.approx(brg_south, abs=0.5) == 180.0

    # Due West
    brg_west = GPSMath.bearing_degrees(15.0, 74.0, 15.0, 73.0)
    assert pytest.approx(brg_west, abs=0.5) == 270.0


def test_heading_error_wrap_around():
    # 359 deg to 1 deg (should be +2 deg, NOT -358 deg)
    err1 = GPSMath.heading_error_degrees(359.0, 1.0)
    assert pytest.approx(err1, abs=0.01) == 2.0

    # 1 deg to 359 deg (should be -2 deg, NOT +358 deg)
    err2 = GPSMath.heading_error_degrees(1.0, 359.0)
    assert pytest.approx(err2, abs=0.01) == -2.0

    # 180 deg reversal
    err3 = GPSMath.heading_error_degrees(90.0, 270.0)
    assert abs(abs(err3) - 180.0) < 0.01
