import math

# =========================================================
# UTILITY FUNCTIONS
# =========================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def smooth(current, target, alpha):
    return current + alpha * (target - current)


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi