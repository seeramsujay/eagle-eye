"""
Project Eagle-Eye: 2D Constant-Velocity Kalman Filter for Aerial Tracking
State Vector: x = [cx, cy, Area, vx, vy, vArea]^T
Measurement Vector: z = [cx, cy, Area]^T
"""

import numpy as np
from typing import Tuple, Optional


class ConstantVelocityKalmanTracker:
    def __init__(self, init_cx: float, init_cy: float, init_area: float,
                 q_pos: float = 1.0, q_vel: float = 5.0,
                 r_pos: float = 4.0, r_area: float = 20.0):
        """
        Initializes 6-state constant velocity Kalman filter
        """
        # State vector: [cx, cy, A, vx, vy, vA]
        self.x = np.array([init_cx, init_cy, init_area, 0.0, 0.0, 0.0], dtype=np.float64)

        # State Covariance matrix P
        self.P = np.diag([10.0, 10.0, 50.0, 100.0, 100.0, 200.0]).astype(np.float64)

        # Process Noise Covariance Q
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.Q = np.diag([q_pos, q_pos, q_pos * 2.0, q_vel, q_vel, q_vel * 2.0]).astype(np.float64)

        # Measurement Noise Covariance R
        self.R = np.diag([r_pos, r_pos, r_area]).astype(np.float64)

        # Measurement Matrix H (3x6)
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0 # cx
        self.H[1, 1] = 1.0 # cy
        self.H[2, 2] = 1.0 # Area

        self.time_since_update = 0.0
        self.total_updates = 1

    def predict(self, dt: float) -> np.ndarray:
        """
        Predicts state ahead by dt seconds using constant-velocity model
        """
        if dt <= 0:
            return self.x.copy()

        # State transition matrix F (6x6)
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt # cx += vx * dt
        F[1, 4] = dt # cy += vy * dt
        F[2, 5] = dt # Area += vA * dt

        # x = F * x
        self.x = F @ self.x

        # Area cannot be negative
        if self.x[2] < 10.0:
            self.x[2] = 10.0
            self.x[5] = 0.0 # reset area velocity

        # P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + (self.Q * dt)
        self.time_since_update += dt
        return self.x.copy()

    def update(self, cx: float, cy: float, area: float):
        """
        Kalman update step with new measurement z = [cx, cy, area]
        """
        z = np.array([cx, cy, area], dtype=np.float64)

        # Innovation: y = z - H * x
        y = z - (self.H @ self.x)

        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman Gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Updated state estimate: x = x + K * y
        self.x = self.x + (K @ y)

        # Ensure positive area
        if self.x[2] < 10.0:
            self.x[2] = 10.0

        # Updated covariance estimate: P = (I - K * H) * P
        I = np.eye(6, dtype=np.float64)
        self.P = (I - (K @ self.H)) @ self.P

        self.time_since_update = 0.0
        self.total_updates += 1

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[3]), float(self.x[4])

    @property
    def area(self) -> float:
        return float(self.x[2])

    def get_predicted_bbox(self, aspect_ratio: float = 1.6) -> Tuple[int, int, int, int]:
        """
        Returns estimated (x1, y1, x2, y2) bounding box from (cx, cy, Area)
        w * h = Area, w / h = aspect_ratio -> w = sqrt(Area * aspect_ratio), h = w / aspect_ratio
        """
        area = max(10.0, float(self.x[2]))
        w = np.sqrt(area * aspect_ratio)
        h = w / aspect_ratio
        cx, cy = self.x[0], self.x[1]
        x1 = int(cx - w / 2.0)
        y1 = int(cy - h / 2.0)
        x2 = int(cx + w / 2.0)
        y2 = int(cy + h / 2.0)
        return (x1, y1, x2, y2)
