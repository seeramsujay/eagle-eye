import os
import sys

import pytest

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from lora_link import (
    FLAG_ARMED,
    FLAG_GPS_FIX,
    MODE_MANUAL_FPV,
    MODE_VISION_TRACK,
    DownlinkPacket,
    LoRaLink,
    UplinkPacket,
    calculate_crc16,
)


def test_packet_sizes():
    assert UplinkPacket.SIZE == 16, f"UplinkPacket must be exactly 16 bytes, got {UplinkPacket.SIZE}"
    assert DownlinkPacket.SIZE == 16, f"DownlinkPacket must be exactly 16 bytes, got {DownlinkPacket.SIZE}"


def test_crc16_integrity():
    data = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e"
    crc = calculate_crc16(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF

    # Verify deterministic calculation
    assert calculate_crc16(data) == crc
    # Any single bit flip must change CRC
    data_corrupt = b"\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0f"
    assert calculate_crc16(data_corrupt) != crc


def test_uplink_pack_unpack_roundtrip():
    original = UplinkPacket(
        mode=MODE_VISION_TRACK,
        roll_deg=22.5,
        pitch_deg=-3.4,
        throttle_pct=72,
        car_lat=15.4909123,
        car_lon=73.8278456,
    )
    packed = original.pack()
    assert len(packed) == 16

    unpacked = UplinkPacket.unpack(packed)
    assert unpacked is not None
    assert unpacked.mode == MODE_VISION_TRACK
    assert pytest.approx(unpacked.roll_deg, abs=0.02) == 22.5
    assert pytest.approx(unpacked.pitch_deg, abs=0.02) == -3.4
    assert unpacked.throttle_pct == 72
    assert pytest.approx(unpacked.car_lat, abs=1e-6) == 15.4909123
    assert pytest.approx(unpacked.car_lon, abs=1e-6) == 73.8278456


def test_uplink_corruption_rejection():
    pkt = UplinkPacket(mode=MODE_MANUAL_FPV, roll_deg=10.0, pitch_deg=5.0, throttle_pct=50)
    packed = bytearray(pkt.pack())
    # Corrupt a payload byte
    packed[2] ^= 0xFF
    assert UplinkPacket.unpack(bytes(packed)) is None


def test_downlink_pack_unpack_roundtrip():
    original = DownlinkPacket(
        plane_lat=15.4920000,
        plane_lon=73.8290000,
        alt_m=58.0,
        speed_kmh=48.0,
        heading_deg=184.0,
        batt_v=11.4,
        state_flags=FLAG_ARMED | FLAG_GPS_FIX,
    )
    packed = original.pack()
    assert len(packed) == 16

    unpacked = DownlinkPacket.unpack(packed)
    assert unpacked is not None
    assert pytest.approx(unpacked.plane_lat, abs=1e-6) == 15.4920000
    assert pytest.approx(unpacked.plane_lon, abs=1e-6) == 73.8290000
    assert unpacked.alt_m == 58.0
    assert unpacked.speed_kmh == 48.0
    assert pytest.approx(unpacked.heading_deg, abs=2.0) == 184.0
    assert pytest.approx(unpacked.batt_v, abs=0.1) == 11.4
    assert unpacked.state_flags == (FLAG_ARMED | FLAG_GPS_FIX)


def test_downlink_corruption_rejection():
    pkt = DownlinkPacket(plane_lat=15.0, plane_lon=73.0, alt_m=100.0, speed_kmh=50.0)
    packed = bytearray(pkt.pack())
    packed[0] ^= 0x55
    assert DownlinkPacket.unpack(bytes(packed)) is None


def test_lora_mock_link_throughput():
    link = LoRaLink(mock_mode=True)
    assert link.connect() is True

    received_packets = []
    link.downlink_callback = lambda p: received_packets.append(p)

    for i in range(25):
        up = UplinkPacket(mode=MODE_VISION_TRACK, roll_deg=float(i), throttle_pct=60)
        link.send_uplink(up)

    assert link.packets_sent == 25
    assert link.packets_received == 25
    assert len(received_packets) == 25
    assert link.crc_errors == 0
    link.disconnect()
