"""Pose factory for tests that need "the vehicle is HERE, facing THERE" without
running the estimator (that has its own tests in test_estimator.py)."""
from __future__ import annotations

from bahr_pilot.estimator import EstimateStatus, Pose


def make_pose(lat: float | None = 41.0, lon: float | None = 29.0, heading_deg: float = 0.0, *,
              heading_valid: bool = True, speed_mps: float = 0.0, course_deg: float | None = None,
              velocity_en: tuple[float, float] = (0.0, 0.0), yaw_rate_dps: float = 0.0, t: float = 0.0,
              sigma_m: float | None = 0.05, roll_deg: float | None = None, pitch_deg: float | None = None,
              status: EstimateStatus | None = None) -> Pose:
    if status is None:
        status = EstimateStatus.OK if lat is not None else EstimateStatus.NONE
    return Pose(
        t=t, status=status, lat=lat, lon=lon, east_m=0.0, north_m=0.0, position_sigma_m=sigma_m,
        velocity_east_mps=velocity_en[0], velocity_north_mps=velocity_en[1], speed_mps=speed_mps, course_deg=course_deg,
        heading_valid=heading_valid, heading_deg=heading_deg, heading_sigma_deg=0.5,
        gyro_bias_dps=0.0, yaw_rate_dps=yaw_rate_dps,
        roll_deg=roll_deg, pitch_deg=pitch_deg,
    )
