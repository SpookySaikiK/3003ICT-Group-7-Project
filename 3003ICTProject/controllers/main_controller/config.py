import math

# =========================================================
# SYSTEM CONSTANTS
# =========================================================

MAX_SPEED = 8.0
TURN_SPEED = 0.7

THRESHOLD = 100
CLIFF_THRESHOLD = 850

ALPHA = 0.15
CELL_SIZE = 0.45

# Docking configuration
DOCK_TOLERANCE = 0.05
DOCK_LOCK_DISTANCE = 0.5

# Battery model
BATTERY_MAX = 100.0
BATTERY_LOW = 20.0
CONSUMPTION_RATE = 0.1
CHARGE_RATE = 10.0

# Control gains
KP_ANGLE = 4.0
RETURN_SPEED = 0.4 * MAX_SPEED

# Stuck detection thresholds
STUCK_CHECK_INTERVAL = 2.0
STUCK_MIN_DISTANCE = 0.05
STUCK_TIME_THRESHOLD = 10.0

# Recovery timing
RECOVERY_BEEP_INTERVAL = 10.0

# Sensor weighting map (left/right influence)
WEIGHTS = [
    (150,0),(200,0),(300,0),(600,0),
    (0,600),(0,300),(0,200),(0,150),
] + [(0,0)] * 8

# =========================================================
# STATE MACHINE
# =========================================================

PAUSED, CLEANING, AVOID, CHARGING, CLIFF, RECOVERY, EXPLORE = range(7)
STATE_NAMES = ('PAUSED', 'CLEANING', 'AVOID', 'CHARGING', 'CLIFF', 'RECOVERY', 'EXPLORE')

STATE_LEDS = {
    'PAUSED':   (),
    'CLEANING': (5,),
    'AVOID':    (0, 1),
    'CHARGING': (4,),
    'CLIFF':    (2, 3, 6),
    'RECOVERY': (0, 1, 2, 3, 6),
    'EXPLORE':  (4, 5),
}