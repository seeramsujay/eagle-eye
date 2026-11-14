"""
Project Eagle-Eye: Binary LoRa Telemetry Protocol & Transport Link
Compatible with Firmware common/protocol.h
"""

import struct
import threading
import time
from collections.abc import Callable
from typing import Optional

# Flight Controller Modes
MODE_DISARMED = 0x00
MODE_MANUAL_FPV = 0x01
MODE_VISION_TRACK = 0x02
MODE_KALMAN_COAST = 0x03
MODE_GPS_PURSUIT = 0x04
MODE_RETURN_TO_CAR = 0x05
MODE_FAILSAFE_BLIND = 0x06

# Plane Flags
FLAG_ARMED = 1 << 0
FLAG_GPS_FIX = 1 << 1
FLAG_LORA_CONNECTED = 1 << 2
FLAG_FAILSAFE_ACTIVE = 1 << 3
FLAG_LOW_BATT = 1 << 4

# Bridge Framing Constants
FRAME_HEADER_1 = 0x55
FRAME_HEADER_2 = 0xAA
FRAME_TYPE_UPLINK = 0x01
FRAME_TYPE_DOWNLINK = 0x02
FRAME_TYPE_PING = 0x03
FRAME_TYPE_PONG = 0x04


def calculate_crc16(data: bytes) -> int:
    """CRC-16-CCITT (Polynomial 0x1021, Init 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class UplinkPacket:
    """
    16-Byte Packed Uplink Command Structure:
    <B   mode (1 byte)
    <h   roll_cmd_cdeg (2 bytes)
    <h   pitch_cmd_cdeg (2 bytes)
    <B   throttle_pct (1 byte)
    <i   car_lat_e7 (4 bytes)
    <i   car_lon_e7 (4 bytes)
    <H   crc16 (2 bytes)
    """

    STRUCT_FMT = "<BhhBiiH"
    SIZE = struct.calcsize(STRUCT_FMT)  # 16 bytes

    def __init__(
        self,
        mode: int = MODE_DISARMED,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        throttle_pct: int = 0,
        car_lat: float = 0.0,
        car_lon: float = 0.0,
    ):
        self.mode = mode
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.throttle_pct = max(0, min(100, int(throttle_pct)))
        self.car_lat = car_lat
        self.car_lon = car_lon

    def pack(self) -> bytes:
        roll_cdeg = max(-3500, min(3500, int(self.roll_deg * 100)))
        pitch_cdeg = max(-1500, min(2000, int(self.pitch_deg * 100)))
        car_lat_e7 = int(self.car_lat * 1e7)
        car_lon_e7 = int(self.car_lon * 1e7)

        # Pack first 14 bytes without CRC
        partial = struct.pack(
            "<BhhBii",
            self.mode,
            roll_cdeg,
            pitch_cdeg,
            self.throttle_pct,
            car_lat_e7,
            car_lon_e7,
        )
        crc = calculate_crc16(partial)
        return partial + struct.pack("<H", crc)

    @classmethod
    def unpack(cls, data: bytes) -> Optional["UplinkPacket"]:
        if len(data) != cls.SIZE:
            return None
        payload, crc_rec = data[:14], struct.unpack("<H", data[14:16])[0]
        if calculate_crc16(payload) != crc_rec:
            return None  # CRC failure
        mode, roll_cdeg, pitch_cdeg, thr, lat_e7, lon_e7 = struct.unpack(
            "<BhhBii", payload
        )
        pkt = cls(
            mode, roll_cdeg / 100.0, pitch_cdeg / 100.0, thr, lat_e7 / 1e7, lon_e7 / 1e7
        )
        return pkt


class DownlinkPacket:
    """
    16-Byte Packed Downlink Telemetry Structure:
    <i   plane_lat_e7 (4 bytes)
    <i   plane_lon_e7 (4 bytes)
    <h   alt_m (2 bytes)
    <B   speed_kmh (1 byte)
    <B   heading_deg_div2 (1 byte)
    <B   batt_v_x10 (1 byte)
    <B   state_flags (1 byte)
    <H   crc16 (2 bytes)
    """

    STRUCT_FMT = "<iihBBBBH"
    SIZE = struct.calcsize(STRUCT_FMT)  # 16 bytes

    def __init__(
        self,
        plane_lat: float = 0.0,
        plane_lon: float = 0.0,
        alt_m: float = 0.0,
        speed_kmh: float = 0.0,
        heading_deg: float = 0.0,
        batt_v: float = 11.1,
        state_flags: int = 0,
    ):
        self.plane_lat = plane_lat
        self.plane_lon = plane_lon
        self.alt_m = alt_m
        self.speed_kmh = speed_kmh
        self.heading_deg = heading_deg
        self.batt_v = batt_v
        self.state_flags = state_flags

    def pack(self) -> bytes:
        lat_e7 = int(self.plane_lat * 1e7)
        lon_e7 = int(self.plane_lon * 1e7)
        alt_m = int(self.alt_m)
        spd = max(0, min(255, int(self.speed_kmh)))
        hdg_div2 = max(0, min(179, int((self.heading_deg % 360.0) / 2.0)))
        batt_x10 = max(0, min(255, int(self.batt_v * 10.0)))
        partial = struct.pack(
            "<iihBBBB", lat_e7, lon_e7, alt_m, spd, hdg_div2, batt_x10, self.state_flags
        )
        crc = calculate_crc16(partial)
        return partial + struct.pack("<H", crc)

    @classmethod
    def unpack(cls, data: bytes) -> Optional["DownlinkPacket"]:
        if len(data) != cls.SIZE:
            return None
        payload, crc_rec = data[:14], struct.unpack("<H", data[14:16])[0]
        if calculate_crc16(payload) != crc_rec:
            return None
        lat_e7, lon_e7, alt_m, spd, hdg_div2, batt_x10, flags = struct.unpack(
            "<iihBBBB", payload
        )
        pkt = cls(
            lat_e7 / 1e7,
            lon_e7 / 1e7,
            float(alt_m),
            float(spd),
            float(hdg_div2 * 2.0),
            batt_x10 / 10.0,
            flags,
        )
        return pkt


class LoRaLink:
    """
    Manages serial bridge communication to SX1278 ground station,
    including packet framing, CRC validation, latency measurement,
    and mock loopback transport for offline simulation.
    """

    def __init__(
        self, port: str | None = None, baudrate: int = 921600, mock_mode: bool = False
    ):
        self.port = port
        self.baudrate = baudrate
        self.mock_mode = mock_mode
        self.serial_handle = None
        self.running = False
        self.rx_thread = None

        self.packets_sent = 0
        self.packets_received = 0
        self.crc_errors = 0
        self.last_downlink: DownlinkPacket | None = None
        self.downlink_callback: Callable[[DownlinkPacket], None] | None = None
        self.lock = threading.Lock()

        # Simulated airplane state for mock mode
        self.mock_plane_lat = 15.4909
        self.mock_plane_lon = 73.8278
        self.mock_plane_alt = 45.0
        self.mock_plane_speed = 42.0
        self.mock_plane_heading = 90.0
        self.mock_batt_v = 11.8

    def connect(self) -> bool:
        if self.mock_mode or self.port is None:
            self.running = True
            return True
        try:
            import serial

            self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.rx_thread.start()
            return True
        except (OSError, ImportError, RuntimeError) as e:
            print(
                f"[LoRaLink] Serial connection failed: {e}. Falling back to mock mode."
            )
            self.mock_mode = True
            self.running = True
            return True

    def disconnect(self):
        self.running = False
        if self.serial_handle and self.serial_handle.is_open:
            self.serial_handle.close()

    def send_uplink(self, packet: UplinkPacket):
        data = packet.pack()
        with self.lock:
            self.packets_sent += 1

        if self.mock_mode:
            # Immediate loopback simulation: generate plausible downlink
            with self.lock:
                self.mock_plane_heading = (
                    self.mock_plane_heading + packet.roll_deg * 0.1
                ) % 360.0
                downlink = DownlinkPacket(
                    plane_lat=self.mock_plane_lat,
                    plane_lon=self.mock_plane_lon,
                    alt_m=self.mock_plane_alt,
                    speed_kmh=self.mock_plane_speed,
                    heading_deg=self.mock_plane_heading,
                    batt_v=self.mock_batt_v,
                    state_flags=FLAG_ARMED | FLAG_GPS_FIX | FLAG_LORA_CONNECTED,
                )
                self.last_downlink = downlink
                self.packets_received += 1
                if self.downlink_callback:
                    self.downlink_callback(downlink)
            return

        # Send framed packet to ground bridge
        # Frame: [0x55, 0xAA, TYPE_UPLINK, LEN(16), Payload(16), CRC16(2)]
        header = bytes([FRAME_HEADER_1, FRAME_HEADER_2, FRAME_TYPE_UPLINK, len(data)])
        frame_crc = struct.pack("<H", calculate_crc16(data))
        if self.serial_handle and self.serial_handle.is_open:
            self.serial_handle.write(header + data + frame_crc)

    def _read_loop(self):
        buffer = bytearray()
        while self.running and self.serial_handle and self.serial_handle.is_open:
            try:
                chunk = self.serial_handle.read(64)
                if chunk:
                    buffer.extend(chunk)
                    while (
                        len(buffer) >= 22
                    ):  # Min frame size = 4 hdr + 16 payload + 2 crc
                        if buffer[0] == FRAME_HEADER_1 and buffer[1] == FRAME_HEADER_2:
                            ftype = buffer[2]
                            flen = buffer[3]
                            if len(buffer) < 4 + flen + 2:
                                break  # Wait for full frame
                            payload = bytes(buffer[4 : 4 + flen])
                            frame_crc = struct.unpack(
                                "<H", buffer[4 + flen : 4 + flen + 2]
                            )[0]
                            if calculate_crc16(payload) == frame_crc:
                                if (
                                    ftype == FRAME_TYPE_DOWNLINK
                                    and flen == DownlinkPacket.SIZE
                                ):
                                    downlink = DownlinkPacket.unpack(payload)
                                    if downlink:
                                        with self.lock:
                                            self.last_downlink = downlink
                                            self.packets_received += 1
                                        if self.downlink_callback:
                                            self.downlink_callback(downlink)
                            else:
                                with self.lock:
                                    self.crc_errors += 1
                            buffer = buffer[4 + flen + 2 :]
                        else:
                            buffer.pop(0)  # Scan forward for header
                else:
                    time.sleep(0.005)
            except (OSError, ValueError):
                time.sleep(0.05)
