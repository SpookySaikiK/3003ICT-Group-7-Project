import math
from config import (
    MAX_SPEED, TURN_SPEED, KP_ANGLE, RETURN_SPEED,
    DOCK_TOLERANCE, DOCK_LOCK_DISTANCE, ALPHA, CELL_SIZE
)
from utils import clamp, wrap_angle, smooth

# =========================================================
# NAVIGATION / DOCKING
# =========================================================

def at_dock(x, z, dock):
    return math.hypot(x - dock[0], z - dock[1]) < DOCK_TOLERANCE


def go_to_dock(x, z, theta, dock):
    dx = dock[0] - x
    dz = dock[1] - z
    dist = math.hypot(dx, dz)

    if dist < DOCK_TOLERANCE:
        return 0.0, 0.0

    target_angle = math.atan2(dz, dx)
    angle_error = wrap_angle(target_angle - theta)

    if abs(angle_error) > 0.2:
        turn = KP_ANGLE * angle_error
        turn = clamp(turn, -MAX_SPEED, MAX_SPEED)
        return -turn, turn

    speed = min(RETURN_SPEED, MAX_SPEED)
    return speed, speed


def follow_left_wall(left_force, right_force):
    base = MAX_SPEED * 0.45
    total = left_force + right_force

    # wall -> wall following behaviour
    if total > 50:
        error = left_force * 1.4 - right_force * 0.5
        Kp = 0.014

        vl = base - Kp * error
        vr = base + Kp * error

    # no wall -> turning bias
    else:
        drift = 0.3

        vl = base * (1 - drift)
        vr = base * (1 + drift)

    return clamp(vl, -MAX_SPEED, MAX_SPEED), clamp(vr, -MAX_SPEED, MAX_SPEED)


def move_to_target(target, x, z, theta):
    tx, tz = target

    target_x = tx * CELL_SIZE
    target_z = tz * CELL_SIZE

    dx = target_x - x
    dz = target_z - z

    distance = math.hypot(dx, dz)

    if distance < 0.1:
        return 0.0, 0.0, True

    target_angle = math.atan2(dz, dx)
    angle_error = wrap_angle(target_angle - theta)

    if abs(angle_error) > 0.2:
        turn = clamp(KP_ANGLE * angle_error, -MAX_SPEED, MAX_SPEED)
        return -turn, turn, False

    speed = 0.5 * MAX_SPEED
    return speed, speed, False