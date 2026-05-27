import math
from config import WEIGHTS, MAX_SPEED, THRESHOLD, CLIFF_THRESHOLD

# =========================================================
# SENSOR REFERENCES
# =========================================================

sensors = []
cliff_left = None
cliff_right = None
gps = None
compass = None
keyboard = None

# =========================================================
# POSE STATE
# =========================================================

x = 0.0
z = 0.0
theta = 0.0

# =========================================================
# INITIALISATION
# =========================================================

def init_sensors(robot, dt_ms):
    global sensors, cliff_left, cliff_right, gps, compass, keyboard
    from controller import Keyboard

    # Obstacle IR sensors (16 channels)
    sensors = [robot.getDevice(f"so{i}") for i in range(16)]
    for sensor in sensors:
        sensor.enable(dt_ms)

    # Cliff detection sensors
    cliff_left = robot.getDevice("cliff_left")
    cliff_right = robot.getDevice("cliff_right")
    cliff_left.enable(dt_ms)
    cliff_right.enable(dt_ms)

    # Localisation sensors
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    gps.enable(dt_ms)
    compass.enable(dt_ms)

    # Keyboard controller
    keyboard = Keyboard()
    keyboard.enable(dt_ms)


# =========================================================
# SENSOR PROCESSING
# =========================================================

def update_pose():
    global x, z, theta

    gps_values = gps.getValues()
    x, z = gps_values[0], gps_values[1]

    north = compass.getValues()
    theta = _wrap_angle(math.atan2(north[0], north[1]))


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def read_obstacles():
    left_force = 0.0
    right_force = 0.0

    for i, sensor in enumerate(sensors):
        value = sensor.getValue()
        if value <= 0:
            continue

        distance = 5.0 * (1.0 - value / 1024.0)
        if distance >= 0.5:
            continue

        influence = 1.0 - (distance / 0.5)

        wl, wr = WEIGHTS[i]
        left_force += wl * influence
        right_force += wr * influence

    return left_force, right_force


def read_cliff():
    return cliff_left.getValue(), cliff_right.getValue()


def read_keyboard():
    key = keyboard.getKey()

    if key == ord(' '):
        return 'toggle'
    if key == ord('C'):
        return 'charge'
    return None