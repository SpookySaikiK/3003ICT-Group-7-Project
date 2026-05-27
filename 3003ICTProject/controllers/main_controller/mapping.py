import math
from config import CELL_SIZE

# =========================================================
# MAP STATE
# =========================================================

grid = {}
current_target = None

# =========================================================
# GRID VISUALISATION
# =========================================================

def print_grid(x, z, min_x, max_x, min_z, max_z):
    gx0 = int(math.floor(min_x / CELL_SIZE)) - 1
    gx1 = int(math.ceil(max_x / CELL_SIZE)) + 1
    gz0 = int(math.floor(min_z / CELL_SIZE)) - 1
    gz1 = int(math.ceil(max_z / CELL_SIZE)) + 1

    robot_x = int(round(x / CELL_SIZE))
    robot_z = int(round(z / CELL_SIZE))

    for gz in range(gz1, gz0 - 1, -1):
        row = ""
        for gx in range(gx0, gx1 + 1):

            if gx in (gx0, gx1) or gz in (gz0, gz1):
                row += "="
                continue

            cell = (gx, gz)

            if gx == robot_x and gz == robot_z:
                row += "R"
                continue

            row += grid.get(cell, '.')

        print(row)


# =========================================================
# MAP PROCESSING
# =========================================================

def finalize_grid(min_x, max_x, min_z, max_z):
    global grid

    gx0 = int(math.floor(min_x / CELL_SIZE)) - 1
    gx1 = int(math.ceil(max_x / CELL_SIZE)) + 1
    gz0 = int(math.floor(min_z / CELL_SIZE)) - 1
    gz1 = int(math.ceil(max_z / CELL_SIZE)) + 1

    # Add borders
    for gx in range(gx0, gx1 + 1):
        grid[(gx, gz0)] = '='
        grid[(gx, gz1)] = '='

    for gz in range(gz0, gz1 + 1):
        grid[(gx0, gz)] = '='
        grid[(gx1, gz)] = '='

    # Mark adjacent cells as walls
    for gz in range(gz0, gz1 + 1):
        for gx in range(gx0, gx1 + 1):
            cell = (gx, gz)

            if grid.get(cell, '.') != '.':
                continue

            if (
                grid.get((gx+1, gz), '.') == '=' or
                grid.get((gx-1, gz), '.') == '=' or
                grid.get((gx, gz+1), '.') == '=' or
                grid.get((gx, gz-1), '.') == '='
            ):
                grid[cell] = '#'

    # Fill expansion
    changed = True
    while changed:
        changed = False
        for gz in range(gz0, gz1 + 1):
            for gx in range(gx0, gx1 + 1):
                cell = (gx, gz)

                if grid.get(cell, '.') != '.':
                    continue

                if (
                    grid.get((gx+1, gz), '.') == '#' or
                    grid.get((gx-1, gz), '.') == '#' or
                    grid.get((gx, gz+1), '.') == '#' or
                    grid.get((gx, gz-1), '.') == '#'
                ):
                    grid[cell] = '#'
                    changed = True


def compute_coverage(min_x, max_x, min_z, max_z):
    visited = 0
    total = 0

    gx0 = int(math.floor(min_x / CELL_SIZE)) - 1
    gx1 = int(math.ceil(max_x / CELL_SIZE)) + 1
    gz0 = int(math.floor(min_z / CELL_SIZE)) - 1
    gz1 = int(math.ceil(max_z / CELL_SIZE)) + 1

    for gz in range(gz0 + 1, gz1):
        for gx in range(gx0 + 1, gx1):

            cell = (gx, gz)
            value = grid.get(cell, '.')

            if value in ('#', '='):
                continue

            total += 1

            if value == '+':
                visited += 1

    if total == 0:
        return 0.0

    return (visited / total) * 100.0


# =========================================================
# TARGET SELECTION
# =========================================================

def find_next_target(min_x, max_x, min_z, max_z):
    gx0 = int(math.floor(min_x / CELL_SIZE)) - 1
    gx1 = int(math.ceil(max_x / CELL_SIZE)) + 1
    gz0 = int(math.floor(min_z / CELL_SIZE)) - 1
    gz1 = int(math.ceil(max_z / CELL_SIZE)) + 1

    for gz in range(gz0 + 1, gz1):

        if gz % 2 != 0:
            gx_range = range(gx0 + 1, gx1)
        else:
            gx_range = range(gx1 - 1, gx0, -1)

        for gx in gx_range:
            if grid.get((gx, gz), '.') == '.':
                return (gx, gz)

    return None


def update_cell(x, z):
    """Mark current cell as visited."""
    cell = (int(round(x / CELL_SIZE)), int(round(z / CELL_SIZE)))
    if cell not in grid or grid[cell] == '.':
        grid[cell] = '+'
    return cell