from controller import Robot
import math

from config import (
    MAX_SPEED, TURN_SPEED, ALPHA, CELL_SIZE,
    BATTERY_MAX, BATTERY_LOW, CONSUMPTION_RATE, CHARGE_RATE,
    KP_ANGLE, DOCK_TOLERANCE, DOCK_LOCK_DISTANCE,
    STUCK_CHECK_INTERVAL, STUCK_MIN_DISTANCE, STUCK_TIME_THRESHOLD,
    RECOVERY_BEEP_INTERVAL, THRESHOLD, CLIFF_THRESHOLD,
    PAUSED, CLEANING, AVOID, CHARGING, CLIFF, RECOVERY, EXPLORE,
    STATE_NAMES, STATE_LEDS
)
from utils import clamp, smooth, wrap_angle
from sensors import (
    init_sensors, update_pose, read_obstacles, read_cliff, read_keyboard,
    x, z, theta
)
import sensors as sens
from navigation import at_dock, go_to_dock, follow_left_wall, move_to_target
from mapping import (
    grid, print_grid, finalize_grid, compute_coverage,
    find_next_target, update_cell
)

# =========================================================
# ROBOT INITIALISATION
# =========================================================

robot = Robot()
dt_ms = int(robot.getBasicTimeStep())
dt = dt_ms / 1000.0

# =========================================================
# ACTUATORS
# =========================================================

# Drive motors (continuous rotation mode)
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# LED array (7 indicators)
leds = [robot.getDevice(f"led{i}") for i in range(7)]

# Speaker output
speaker = robot.getDevice("speaker")

# =========================================================
# SENSORS INIT
# =========================================================

init_sensors(robot, dt_ms)

# =========================================================
# ROBOT STATE VARIABLES
# =========================================================

# Map bounds tracking
min_x = max_x = 0.0
min_z = max_z = 0.0

# Battery state
battery = BATTERY_MAX
charging_complete = True

# Motor smoothing
vl_smooth = vr_smooth = 0.0

# Docking state
dock_initialized = False
DOCK = (0.0, 0.0)

# Cliff behaviour tracking
cliff_timer = 0.0
cliff_phase = 0
cliff_dir = 1

# Stuck detection tracking
stuck_timer = 0.0
stuck_check = 0.0
last_check_x = 0.0
last_check_z = 0.0

# Recovery timing
recovery_beep_timer = 0.0

# Exploration tracking
exploration_complete = False
exploration_timer = 0.0

# State machine
state = EXPLORE
turn_dir = 1

# Cleaning target
current_target = None

# =========================================================
# LOCAL HELPERS
# =========================================================

def update_leds():
    active_leds = STATE_LEDS[STATE_NAMES[state]]
    for i, led in enumerate(leds):
        if i in active_leds:
            led.set(1)
        else:
            led.set(0)


def check_stuck():
    global stuck_check, stuck_timer, last_check_x, last_check_z
    x, z = sens.x, sens.z

    stuck_check += dt

    if stuck_check < STUCK_CHECK_INTERVAL:
        return False

    distance_moved = math.hypot(x - last_check_x, z - last_check_z)

    if distance_moved < STUCK_MIN_DISTANCE:
        stuck_timer += stuck_check
    else:
        stuck_timer = 0.0

    last_check_x = x
    last_check_z = z
    stuck_check = 0.0

    return stuck_timer >= STUCK_TIME_THRESHOLD


# =========================================================
# MAIN CONTROL LOOP
# =========================================================

while robot.step(dt_ms) != -1:

    # -----------------------------------------------------
    # SENSOR UPDATE
    # -----------------------------------------------------
    update_pose()
    x, z, theta = sens.x, sens.z, sens.theta

    min_x = min(min_x, x)
    max_x = max(max_x, x)
    min_z = min(min_z, z)
    max_z = max(max_z, z)

    update_cell(x, z)

    left_obs, right_obs = read_obstacles()
    obstacle_detected = left_obs > THRESHOLD or right_obs > THRESHOLD

    cliff_left_value, cliff_right_value = read_cliff()

    cliff_detected = (
        cliff_left_value < CLIFF_THRESHOLD or
        cliff_right_value < CLIFF_THRESHOLD
    )

    keyboard_event = read_keyboard()

    update_leds()

    if not dock_initialized:
        DOCK = (x, z)
        dock_initialized = True

    if not exploration_complete:
        dx = DOCK[0] - x
        dz = DOCK[1] - z
        dist = math.hypot(dx, dz)

        if exploration_timer >= 10 and dist < DOCK_LOCK_DISTANCE and not exploration_complete:
            finalize_grid(min_x, max_x, min_z, max_z)
            exploration_complete = True

    # -----------------------------------------------------
    # STATE TRANSITIONS
    # -----------------------------------------------------

    if cliff_detected and state != CLIFF and state != RECOVERY:
        state = CLIFF
        cliff_timer = 0.0
        cliff_phase = 0
        cliff_dir = 1 if cliff_left_value < cliff_right_value else -1

    if check_stuck() and state not in (RECOVERY, CHARGING, PAUSED):
        state = RECOVERY

    if battery <= BATTERY_LOW and state not in (CHARGING, RECOVERY, CLIFF, AVOID):
        charging_complete = False
        state = CHARGING

    # -----------------------------------------------------
    # STATE MACHINE LOGIC
    # -----------------------------------------------------

    if state == PAUSED:

        if keyboard_event == 'toggle' and exploration_complete and charging_complete:
            stuck_timer = 0.0
            state = CLEANING
        elif keyboard_event == 'toggle' and charging_complete:
            stuck_timer = 0.0
            state = EXPLORE
        elif keyboard_event == 'toggle':
            stuck_timer = 0.0
            state = CHARGING

        if at_dock(x, z, DOCK):
            battery = min(BATTERY_MAX, battery + CHARGE_RATE * dt)
            if battery >= BATTERY_MAX:
                charging_complete = True

    elif state == CLEANING:

        if keyboard_event == 'toggle':
            state = PAUSED
        elif keyboard_event == 'charge' or compute_coverage(min_x, max_x, min_z, max_z) >= 99.99:
            state = CHARGING
        elif obstacle_detected:
            turn_dir = 1 if left_obs >= right_obs else -1
            state = AVOID

    elif state == AVOID:

        if not obstacle_detected and exploration_complete and charging_complete:
            state = CLEANING
        elif not obstacle_detected and charging_complete:
            state = EXPLORE
        elif not obstacle_detected and not charging_complete:
            state = CHARGING

    elif state == CHARGING:

        if at_dock(x, z, DOCK):
            battery = min(BATTERY_MAX, battery + CHARGE_RATE * dt)
            if battery >= BATTERY_MAX:
                charging_complete = True
                stuck_timer = 0.0
                state = PAUSED

    elif state == CLIFF:

        cliff_timer += dt

        if cliff_phase == 0:
            vl = vr = -0.5 * MAX_SPEED
            if cliff_timer >= 2.0:
                cliff_phase = 1
                cliff_timer = 0.0
        else:
            vl = TURN_SPEED * MAX_SPEED * cliff_dir
            vr = -vl
            if cliff_timer >= 1.0 and exploration_complete and charging_complete:
                state = CLEANING
            elif cliff_timer >= 1.0 and charging_complete:
                state = EXPLORE
            elif cliff_timer >= 1.0:
                state = CHARGING

    elif state == RECOVERY:
        stuck_timer = 0.0
        if keyboard_event == 'toggle' and exploration_complete and charging_complete:
            state = CLEANING
        elif keyboard_event == 'toggle' and charging_complete:
            state = EXPLORE
        elif keyboard_event == 'toggle':
            state = CHARGING

    elif state == EXPLORE:

        exploration_timer += dt

        if keyboard_event == 'toggle':
            state = PAUSED
        elif keyboard_event == 'charge':
            state = CHARGING
        elif exploration_complete and charging_complete:
            state = CLEANING
        elif not charging_complete:
            state = CHARGING
        elif obstacle_detected:
            turn_dir = 1
            state = AVOID

    # -----------------------------------------------------
    # MOTOR CONTROL OUTPUT
    # -----------------------------------------------------

    if state == PAUSED:
        vl = vr = 0.0

    elif state == CLEANING:

        if current_target is None:
            current_target = find_next_target(min_x, max_x, min_z, max_z)

        if current_target is None:
            vl = vr = 0.0
        else:
            vl, vr, reached = move_to_target(current_target, x, z, theta)

            if reached:
                current_target = None

    elif state == AVOID:
        vl = TURN_SPEED * MAX_SPEED * turn_dir
        vr = -vl

    elif state == CHARGING:

        dx = DOCK[0] - x
        dz = DOCK[1] - z
        dist = math.hypot(dx, dz)

        if at_dock(x, z, DOCK):
            vl, vr = 0.0, 0.0
            battery = min(BATTERY_MAX, battery + CHARGE_RATE * dt)
            if battery >= BATTERY_MAX:
                state = PAUSED

        elif dist < DOCK_LOCK_DISTANCE:
            vl, vr = go_to_dock(x, z, theta, DOCK)

        elif obstacle_detected:
            turn_dir = 1
            state = AVOID

        else:
            vl, vr = follow_left_wall(left_obs, right_obs)

    if state == RECOVERY:
        vl = vr = 0.0

    elif state == EXPLORE:
        vl, vr = follow_left_wall(left_obs, right_obs)

    # -----------------------------------------------------
    # BATTERY MODEL
    # -----------------------------------------------------

    if state in (CLEANING, AVOID):
        battery = max(0.0, battery - CONSUMPTION_RATE * dt)

    elif state == CHARGING and not at_dock(x, z, DOCK):
        battery = max(0.0, battery - (CONSUMPTION_RATE * 0.5) * dt)

    # Recovery beep logic
    if state == RECOVERY:

        recovery_beep_timer += dt

        if recovery_beep_timer >= RECOVERY_BEEP_INTERVAL:

            speaker.playSound(
                speaker,
                speaker,
                "beep.wav",
                1.0,
                1.0,
                0.0,
                False
            )

            recovery_beep_timer = 0.0

    else:
        recovery_beep_timer = 0.0

    # -----------------------------------------------------
    # MOTOR OUTPUT SMOOTHING
    # -----------------------------------------------------

    vl_smooth = smooth(vl_smooth, vl, ALPHA)
    vr_smooth = smooth(vr_smooth, vr, ALPHA)

    left_motor.setVelocity(vl_smooth)
    right_motor.setVelocity(vr_smooth)

    # -----------------------------------------------------
    # DEBUG OUTPUT
    # -----------------------------------------------------

    print(
        f"State={STATE_NAMES[state]} "
        f"Battery={battery:.1f}%"
    )

    print(
        f"Pos=({x:.2f}, {z:.2f}) "
        f"Obstacle={obstacle_detected}"
    )

    print(
        f"vl={vl_smooth:.2f} vr={vr_smooth:.2f}"
    )

    print_grid(x, z, min_x, max_x, min_z, max_z)
    print(f"Coverage={compute_coverage(min_x, max_x, min_z, max_z):.2f}%")
    print("-" * 40)
    