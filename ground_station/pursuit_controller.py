"""
Project Eagle-Eye: Autonomous Pursuit Controller & State Machine
Implements 3 Pursuit States:
- State 1: VISION_TRACK (PID Visual Servoing)
- State 2: KALMAN_COAST (Target Lost 0-2.5s, Smooth Orbit Decay)
- State 3: GPS_PURSUIT  (Target Lost >2.5s, Great-Circle Intercept until 10-frame re-lock)
"""

import math
from typing import Dict, Any, Tuple
from lora_link import (
    MODE_VISION_TRACK, MODE_KALMAN_COAST, MODE_GPS_PURSUIT,
    MODE_MANUAL_FPV, MODE_DISARMED, UplinkPacket, DownlinkPacket
)


class GPSMath:
    EARTH_RADIUS_M = 6371000.0

    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2.0) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2.0) ** 2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return GPSMath.EARTH_RADIUS_M * c

    @staticmethod
    def bearing_degrees(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
        dlon_r = math.radians(lon2 - lon1)
        y = math.sin(dlon_r) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon_r)
        b_deg = math.degrees(math.atan2(y, x))
        return (b_deg + 360.0) % 360.0

    @staticmethod
    def heading_error_degrees(current_heading: float, target_bearing: float) -> float:
        diff = target_bearing - current_heading
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff


class PIDVisualServoing:
    def __init__(self, kp: float, ki: float, kd: float, min_val: float, max_val: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_val = min_val
        self.max_val = max_val
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self.integral = max(-100.0, min(100.0, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_val, min(self.max_val, output))


class PursuitController:
    def __init__(self, target_box_area: float = 12000.0):
        self.target_box_area = target_box_area
        self.current_state = MODE_VISION_TRACK

        # Visual Servoing PIDs
        # Error normalized to [-1.0, 1.0]
        # Roll: maps dx -> [-35, +35] deg
        self.pid_roll = PIDVisualServoing(kp=35.0, ki=0.8, kd=4.0, min_val=-35.0, max_val=35.0)
        # Pitch: maps dy -> [-15, +20] deg (inverted: target below center -> pitch down/up)
        self.pid_pitch = PIDVisualServoing(kp=20.0, ki=0.5, kd=2.5, min_val=-15.0, max_val=20.0)
        # Throttle: maps area error -> [35, 85] %
        self.pid_throttle = PIDVisualServoing(kp=0.003, ki=0.0001, kd=0.0005, min_val=-25.0, max_val=25.0)

        # Coasting parameters
        self.coast_start_time = 0.0
        self.coast_initial_roll = 0.0
        self.coast_duration_limit = 2.5 # seconds before GPS fallback

        # Relock requirement
        self.relock_frame_count = 0
        self.REQUIRED_RELOCK_FRAMES = 10

    def reset(self):
        self.current_state = MODE_VISION_TRACK
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_throttle.reset()
        self.relock_frame_count = 0

    def update(self, tracking_info: Dict[str, Any], plane_telemetry: DownlinkPacket,
               car_lat: float, car_lon: float, dt: float) -> UplinkPacket:
        """
        Executes 3-state pursuit logic and produces commanded UplinkPacket
        """
        is_detected = tracking_info.get("detected", False)
        time_since_detect = tracking_info.get("time_since_detect", 999.0)

        # -----------------------------------------------------------------
        # State Transitions
        # -----------------------------------------------------------------
        if self.current_state == MODE_VISION_TRACK:
            if not is_detected:
                # Transition to KALMAN_COAST
                self.current_state = MODE_KALMAN_COAST
                self.coast_start_time = 0.0
                self.coast_initial_roll = self.pid_roll.prev_error * self.pid_roll.kp

        elif self.current_state == MODE_KALMAN_COAST:
            if is_detected:
                # Re-locked immediately
                self.current_state = MODE_VISION_TRACK
            elif time_since_detect > self.coast_duration_limit:
                # Target lost > 2.5 seconds -> Fallback to GPS_PURSUIT
                self.current_state = MODE_GPS_PURSUIT
                self.relock_frame_count = 0

        elif self.current_state == MODE_GPS_PURSUIT:
            if is_detected:
                self.relock_frame_count += 1
                if self.relock_frame_count >= self.REQUIRED_RELOCK_FRAMES:
                    # Target consistently re-acquired for 10 frames -> return to VISION_TRACK
                    self.current_state = MODE_VISION_TRACK
                    self.relock_frame_count = 0
            else:
                self.relock_frame_count = 0

        # -----------------------------------------------------------------
        # State Action Execution
        # -----------------------------------------------------------------
        cmd_roll = 0.0
        cmd_pitch = 2.0
        cmd_throttle = 55

        if self.current_state == MODE_VISION_TRACK:
            # PID Visual Servoing
            err_x_norm = tracking_info.get("error_x_norm", 0.0)
            err_y_norm = tracking_info.get("error_y_norm", 0.0)
            box_area = tracking_info.get("area", self.target_box_area)

            # Bank into target: positive error (target to the right) -> bank right (+)
            cmd_roll = self.pid_roll.update(err_x_norm, dt)

            # Pitch: positive error (target below center) -> slight nose down
            cmd_pitch = -self.pid_pitch.update(err_y_norm, dt) + 2.0 # baseline 2° cruise pitch
            cmd_pitch = max(-15.0, min(20.0, cmd_pitch))

            # Throttle: area error = target_area - measured_area
            # If box is too small (plane too far) -> increase throttle
            area_err = self.target_box_area - box_area
            throttle_adjust = self.pid_throttle.update(area_err, dt)
            cmd_throttle = int(max(35, min(85, 55 + throttle_adjust)))

        elif self.current_state == MODE_KALMAN_COAST:
            # Target lost 0-2.5s:
            # Smoothly decays bank angle toward a steady 20° orbit while extrapolating target via Kalman
            decay_factor = min(1.0, time_since_detect / self.coast_duration_limit)
            target_orbit_roll = 20.0 # Standard 20° orbit bank
            cmd_roll = (1.0 - decay_factor) * self.coast_initial_roll + decay_factor * target_orbit_roll
            cmd_pitch = 3.0 # Maintain slight positive pitch for orbit
            cmd_throttle = 55 # Maintain steady cruising speed (30-60 km/h)

        elif self.current_state == MODE_GPS_PURSUIT:
            # Target lost > 2.5s:
            # Ingest live car GPS and plane telemetry to compute intercept roll command
            if car_lat != 0.0 and car_lon != 0.0 and plane_telemetry.plane_lat != 0.0:
                dist_m = GPSMath.haversine_distance(plane_telemetry.plane_lat, plane_telemetry.plane_lon, car_lat, car_lon)
                target_bearing = GPSMath.bearing_degrees(plane_telemetry.plane_lat, plane_telemetry.plane_lon, car_lat, car_lon)
                heading_err = GPSMath.heading_error_degrees(plane_telemetry.heading_deg, target_bearing)

                if dist_m > 80.0:
                    # Coordinated turn toward car bearing
                    cmd_roll = max(-35.0, min(35.0, heading_err * 0.7))
                    cmd_throttle = 70 # Faster intercept throttle
                    cmd_pitch = 3.0
                else:
                    # Overhead orbit: 22° bank circle
                    cmd_roll = 22.0
                    cmd_throttle = 55
                    cmd_pitch = 3.0
            else:
                # Standby blind orbit if GPS coordinates invalid
                cmd_roll = 18.0
                cmd_pitch = 3.0
                cmd_throttle = 45

        return UplinkPacket(
            mode=self.current_state,
            roll_deg=cmd_roll,
            pitch_deg=cmd_pitch,
            throttle_pct=cmd_throttle,
            car_lat=car_lat,
            car_lon=car_lon
        )
