"""Quaternion / Euler / mounting-rotation helpers."""
from __future__ import annotations

import math
import random

import pytest

from bahr_pilot.attitude import (
    IDENTITY, euler_to_quat, mount_quaternion, quat_conjugate, quat_multiply, quat_normalize,
    quat_to_euler, rotate_vector, vehicle_orientation, vehicle_vector,
)


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_identity_has_zero_euler_angles():
    assert quat_to_euler(IDENTITY) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("roll,pitch,yaw", [
    (10, 0, 0), (0, 20, 0), (0, 0, 30), (-15, 25, 100), (170, -45, -170), (0, 89, 0),
])
def test_euler_round_trip(roll, pitch, yaw):
    angles = tuple(math.radians(a) for a in (roll, pitch, yaw))
    recovered = quat_to_euler(euler_to_quat(*angles))
    for got, want in zip(recovered, angles):
        # compare as angles so +-180 deg is the same attitude
        assert math.isclose(math.sin(got - want), 0.0, abs_tol=1e-9)


def test_random_round_trips_never_blow_up_near_gimbal_lock():
    rng = random.Random(5)
    for _ in range(2000):
        q = quat_normalize(tuple(rng.uniform(-1, 1) for _ in range(4)))
        roll, pitch, yaw = quat_to_euler(q)
        assert -math.pi <= roll <= math.pi and -math.pi / 2 <= pitch <= math.pi / 2
        assert not any(math.isnan(a) for a in (roll, pitch, yaw))


def test_known_rotations_move_vectors_the_right_way():
    # +90 deg about z takes x (forward) to y (starboard); right-handed with z DOWN
    q = euler_to_quat(0.0, 0.0, math.radians(90))
    assert close(rotate_vector(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))
    # +90 deg roll takes y (starboard) to z (down)
    q = euler_to_quat(math.radians(90), 0.0, 0.0)
    assert close(rotate_vector(q, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))
    # +90 deg pitch (bow up) takes x (forward) to -z (up)
    q = euler_to_quat(0.0, math.radians(90), 0.0)
    assert close(rotate_vector(q, (1.0, 0.0, 0.0)), (0.0, 0.0, -1.0))


def test_quaternion_composition_and_conjugate():
    a = euler_to_quat(0.1, 0.2, 0.3)
    b = euler_to_quat(-0.3, 0.1, 0.5)
    assert close(quat_multiply(a, quat_conjugate(a)), IDENTITY)
    v = (1.0, 2.0, 3.0)
    assert close(rotate_vector(quat_multiply(a, b), v), rotate_vector(a, rotate_vector(b, v)))


def test_unit_length_is_restored():
    q = quat_normalize((0.0, 0.0, 0.0, 2.0))
    assert q == IDENTITY
    assert quat_normalize((0.0, 0.0, 0.0, 0.0)) == IDENTITY  # garbage in, identity out, no NaN


@pytest.mark.parametrize("orientation", [0, 2, 4, 6, 8])
def test_mounting_rotation_inverts_cleanly(orientation):
    """Rotating a vehicle-frame vector into the sensor frame and back must
    be the identity — whatever the sign convention, it has to be a proper
    rotation and consistent with vehicle_vector()."""
    v = (1.0, -2.0, 0.5)
    sensor = rotate_vector(mount_quaternion(orientation), v)
    assert close(vehicle_vector(sensor, orientation), v)


def test_mounting_documented_examples():
    # board rotated +90 deg about z relative to the vehicle: the vehicle's x
    # axis shows up on the board's y axis, so a board reading (0, 1, 0) is
    # the vehicle moving forward
    assert close(vehicle_vector((0.0, 1.0, 0.0), 2), (1.0, 0.0, 0.0))
    assert close(vehicle_vector((1.0, 0.0, 0.0), 4), (-1.0, 0.0, 0.0))     # yaw 180
    assert close(vehicle_vector((0.0, 0.0, 1.0), 8), (0.0, 0.0, -1.0))     # roll 180: z flips
    assert close(vehicle_vector((0.3, 0.4, 0.5), 0), (0.3, 0.4, 0.5))      # none


def test_unknown_orientation_means_no_rotation():
    assert mount_quaternion(99) == IDENTITY
    assert close(vehicle_vector((1.0, 2.0, 3.0), 99), (1.0, 2.0, 3.0))


def test_vehicle_orientation_with_a_flat_board_is_unchanged():
    q = euler_to_quat(0.1, -0.2, 0.7)
    assert close(vehicle_orientation(q, 0), quat_normalize(q))


def test_a_board_mounted_upside_down_reports_inverted_roll():
    """Board flipped 180 deg about x (AHRS_ORIENTATION 8): when the boat is
    level the board's own attitude is roll 180, and after the mount rotation
    the vehicle must come out level."""
    board_attitude = euler_to_quat(math.pi, 0.0, 0.0)
    roll, pitch, _yaw = quat_to_euler(vehicle_orientation(board_attitude, 8))
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_a_board_yawed_90_on_a_level_boat_gives_zero_roll_and_pitch():
    board_attitude = euler_to_quat(0.0, 0.0, math.radians(90))
    roll, pitch, _ = quat_to_euler(vehicle_orientation(board_attitude, 2))
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9
