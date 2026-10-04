"""Quaternion / Euler helpers and IMU mounting rotation (Phase 4).

Conventions (docs/SENSOR_INTERFACE.md):
  * Quaternions are (x, y, z, w), unit length, Hamilton, and describe the
    rotation WORLD <- BODY: v_world = R(q) * v_body.
  * Body frame: x forward, y starboard (right), z down. Euler angles are the
    aerospace ZYX set: roll about x, pitch about y, yaw about z.
  * The BNO086 reports in ITS OWN frame, which is the vehicle frame only if
    the board is mounted flat with its x axis forward. AHRS_ORIENTATION (the
    ArduPilot parameter BAHR-GCS already lists as "Kart yonu") says how the
    board sits relative to the vehicle:  v_sensor = R_mount * v_vehicle.

Not verified on hardware: the mount-rotation sign convention follows the
statement above, and nothing here has seen a real BNO086. The BNO086 game
rotation vector has an arbitrary, drifting yaw (no magnetometer), so only
roll and pitch from it mean anything; heading comes from GNSS.
"""
from __future__ import annotations

import math

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)

# ArduPilot rotation enum values BAHR-GCS offers (gcs/param_meta.py
# _ROTATION_CHOICES): angle about the named axis, in degrees.
_MOUNT_ROTATIONS: dict[int, tuple[str, float]] = {
    0: ("z", 0.0),      # None
    2: ("z", 90.0),     # Yaw 90
    4: ("z", 180.0),    # Yaw 180
    6: ("z", 270.0),    # Yaw 270
    8: ("x", 180.0),    # Roll 180
}


def quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_normalize(q: Quat) -> Quat:
    norm = math.sqrt(sum(c * c for c in q))
    if norm < 1e-9:
        return IDENTITY
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def quat_from_axis_angle(axis: str, degrees: float) -> Quat:
    half = math.radians(degrees) / 2.0
    s, c = math.sin(half), math.cos(half)
    return {"x": (s, 0.0, 0.0, c), "y": (0.0, s, 0.0, c), "z": (0.0, 0.0, s, c)}[axis]


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """R(q) * v."""
    qv: Quat = (v[0], v[1], v[2], 0.0)
    x, y, z, _ = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (x, y, z)


def quat_to_euler(q: Quat) -> tuple[float, float, float]:
    """(roll, pitch, yaw) in radians, aerospace ZYX."""
    x, y, z, w = quat_normalize(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def mount_quaternion(ahrs_orientation: int) -> Quat:
    """Rotation of the board relative to the vehicle (v_sensor = R * v_vehicle).
    Unknown values are treated as 'no rotation' rather than guessing."""
    axis, degrees = _MOUNT_ROTATIONS.get(int(ahrs_orientation), ("z", 0.0))
    return quat_from_axis_angle(axis, degrees)


def vehicle_orientation(sensor_quat: Quat, ahrs_orientation: int) -> Quat:
    """World <- vehicle orientation from the sensor's world <- sensor one:
    R(q_wv) = R(q_ws) * R_mount."""
    return quat_normalize(quat_multiply(sensor_quat, mount_quaternion(ahrs_orientation)))


def vehicle_vector(sensor_vector: Vec3, ahrs_orientation: int) -> Vec3:
    """Rotate a vector measured in the sensor frame into the vehicle frame:
    v_vehicle = R_mount^T * v_sensor."""
    return rotate_vector(quat_conjugate(mount_quaternion(ahrs_orientation)), sensor_vector)
