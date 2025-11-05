"""
world.py
Environment generation for RBE550 HW4 – Autonomous Valet Parking.

Creates a 12×12 grid (3 m per cell) populated with randomized tetromino
obstacles.  Ensures adequate clearances for the largest vehicle (truck
with trailer), clears the start region in the northwest corner, and
defines a parking bay in the southeast corner.
"""
import numpy as np
import math
import random
from vehicles.base import State2D
from geom.polygons import oriented_box



# ========================
# Global configuration
# ========================

GRID_CELL_SIZE_METERS = 3.0          # Each grid cell is 3×3 meters
GRID_SIZE_CELLS = 12                 # 12×12 grid

# Vehicle–dependent clearance design
TRUCK_WIDTH_METERS = 2.0
SAFETY_MARGIN_METERS = 0.5           # Margin on each side of vehicle
MIN_CORRIDOR_WIDTH_METERS = TRUCK_WIDTH_METERS + 2 * SAFETY_MARGIN_METERS  # 3.0 m
MAX_STEERING_ANGLE_RAD = 0.60        # ~34 degrees
TRUCK_WHEELBASE_METERS = 3.4
MIN_TURN_RADIUS_METERS = TRUCK_WHEELBASE_METERS / math.tan(MAX_STEERING_ANGLE_RAD)  # ≈5.0 m

# Tetromino definitions in grid-cell coordinates
TETROMINO_SHAPES = {
    "I": [(0, 0), (1, 0), (2, 0), (3, 0)],
    "O": [(0, 0), (1, 0), (0, 1), (1, 1)],
    "L": [(0, 0), (0, 1), (0, 2), (1, 2)],
    "J": [(1, 0), (1, 1), (1, 2), (0, 2)],
    "T": [(0, 0), (1, 0), (2, 0), (1, 1)],
}

ROTATION_FUNCTIONS = [
    lambda x, y: (x, y),
    lambda x, y: (-y, x),
    lambda x, y: (-x, -y),
    lambda x, y: (y, -x),
]


# ========================
# Helper functions
# ========================

def convert_cells_to_polygons(cell_indices):
    """Convert grid-cell coordinates to rectangular obstacle polygons."""
    obstacle_polygons = []
    for (col, row) in cell_indices:
        center_x = (col + 0.5) * GRID_CELL_SIZE_METERS
        center_y = (row + 0.5) * GRID_CELL_SIZE_METERS
        obstacle_polygons.append(
             oriented_box(
                 (center_x, center_y),
                 GRID_CELL_SIZE_METERS,        # length
                 GRID_CELL_SIZE_METERS,        # width
                 0.0                           # theta (axis-aligned)
             )
         )                               

    return obstacle_polygons


def get_parking_bay_cells():
    """Return a 3×3 block of cells in the southeast corner for the parking bay."""
    return {(GRID_SIZE_CELLS - 3 + i, 0 + j) for i in range(3) for j in range(3)}


def has_valid_clearance(occupied_cells):
    """Check that obstacle layout leaves sufficient boundary and corridor clearances."""
    boundary_buffer_cells = max(1, math.ceil(MIN_CORRIDOR_WIDTH_METERS / GRID_CELL_SIZE_METERS))


    # Reject obstacles that touch outer boundary
    for (col, row) in occupied_cells:
        if col < boundary_buffer_cells or col >= GRID_SIZE_CELLS - boundary_buffer_cells:
            return False
        if row < boundary_buffer_cells or row >= GRID_SIZE_CELLS - boundary_buffer_cells:
            return False

    # Simple choke-point rejection: forbid 3-of-4 filled 2×2 blocks
    occupied_grid = [[False] * GRID_SIZE_CELLS for _ in range(GRID_SIZE_CELLS)]
    for (col, row) in occupied_cells:
        occupied_grid[row][col] = True
    for col in range(GRID_SIZE_CELLS - 1):
        for row in range(GRID_SIZE_CELLS - 1):
            block = [occupied_grid[row][col],
                     occupied_grid[row][col + 1],
                     occupied_grid[row + 1][col],
                     occupied_grid[row + 1][col + 1]]
            if sum(block) == 3:
                return False

    # Basic turning-radius check near the bay: require at least a 2×2 open block west of bay
    parking_bay_cells = get_parking_bay_cells()
    min_col = min(c for (c, _) in parking_bay_cells)
    max_row = max(r for (_, r) in parking_bay_cells)
    pad_cells = 2  # 2×3 m = 6 m ≥ 5 m radius proxy
    for col in range(min_col - pad_cells, min_col):
        for row in range(0, max_row + 1):
            if not (0 <= col < GRID_SIZE_CELLS and 0 <= row < GRID_SIZE_CELLS):
                return False
            if (col, row) in occupied_cells:
                return False

    return True


def generate_random_obstacle_cells(obstacle_density, random_seed):
    """
    Generate tetromino obstacles satisfying clearance constraints.

    - 1-cell (3 m) clear ring along outer boundary.
    - 1-cell moat between different tetromino pieces (not within a piece).
    - NW start (2 cells) and SE parking bay reserved.
    - Final validation via has_valid_clearance(..).
    """
    random_generator = random.Random(random_seed)
    target_cell_count = int(round(GRID_SIZE_CELLS ** 2 * obstacle_density))

    
    # Reserve a safe moat for the start (NW), robust even with footprint inflation
    start_clear_cells = {
    (0, GRID_SIZE_CELLS - 1), (1, GRID_SIZE_CELLS - 1),
    (0, GRID_SIZE_CELLS - 2), (1, GRID_SIZE_CELLS - 2),
    }
    parking_bay_cells = get_parking_bay_cells()
    
    
    # Rough capacity clamp so we don't ask for more than can fit with the ring
    BOUNDARY_CLEAR_RING_CELLS = 1
    inner_n = max(0, GRID_SIZE_CELLS - 2 * BOUNDARY_CLEAR_RING_CELLS)
    inner_cells = inner_n * inner_n
    reserved_cells = len(start_clear_cells | parking_bay_cells)
    max_placeable_cells = max(0, inner_cells - reserved_cells)
    target_cell_count = min(target_cell_count, max_placeable_cells)

    # Moat (gap) between different tetromino pieces, measured in cells
    MIN_GAP_BETWEEN_SHAPES = 1

    for _ in range(1000):  # resample attempts
        occupied_cells = set()

        MAX_PLACEMENT_TRIES = 5000
        failed_tries = 0

        while len(occupied_cells) < target_cell_count and failed_tries < MAX_PLACEMENT_TRIES:
            shape = random_generator.choice(list(TETROMINO_SHAPES.values()))
            rotation = random_generator.choice(ROTATION_FUNCTIONS)
            offset_col = random_generator.randrange(0, GRID_SIZE_CELLS)
            offset_row = random_generator.randrange(0, GRID_SIZE_CELLS)

            # Build candidate tetromino in grid coordinates
            new_shape_cells = set()
            for (x, y) in shape:
                rx, ry = rotation(x, y)
                col = offset_col + rx
                row = offset_row + ry
                if not (0 <= col < GRID_SIZE_CELLS and 0 <= row < GRID_SIZE_CELLS):
                    new_shape_cells = None
                    break
                new_shape_cells.add((col, row))
            if not new_shape_cells:
                failed_tries += 1
                continue

            # Skip if overlaps reserved areas or already-placed cells
            if (new_shape_cells & start_clear_cells or
                new_shape_cells & parking_bay_cells or
                new_shape_cells & occupied_cells):
                failed_tries += 1
                continue

            # Keep a clear ring from the outer boundary
            touches_boundary_ring = False
            for (c, r) in new_shape_cells:
                if not (BOUNDARY_CLEAR_RING_CELLS <= c < GRID_SIZE_CELLS - BOUNDARY_CLEAR_RING_CELLS and
                        BOUNDARY_CLEAR_RING_CELLS <= r < GRID_SIZE_CELLS - BOUNDARY_CLEAR_RING_CELLS):
                    touches_boundary_ring = True
                    break
            if touches_boundary_ring:
                failed_tries += 1
                continue

            # Enforce moat: no new cell may be Chebyshev-distance 1 from any existing cell
            too_close_to_existing = False
            for (c, r) in new_shape_cells:
                for dc in (-MIN_GAP_BETWEEN_SHAPES, 0, MIN_GAP_BETWEEN_SHAPES):
                    for dr in (-MIN_GAP_BETWEEN_SHAPES, 0, MIN_GAP_BETWEEN_SHAPES):
                        if (dc, dr) == (0, 0):
                            continue
                        if (c + dc, r + dr) in occupied_cells:
                            too_close_to_existing = True
                            break
                    if too_close_to_existing:
                        break
                if too_close_to_existing:
                    break
            if too_close_to_existing:
                failed_tries += 1
                continue

            # Accept this tetromino
            occupied_cells |= new_shape_cells

        # Ensure parking bay is clear
        occupied_cells -= parking_bay_cells
        
        
    # --- Flanking obstacles but leave an entrance gap centered on bay ---
    # parking_bay_cells is a set of (col,row)
    bay_cols = sorted({c for (c, r) in parking_bay_cells})
    bay_rows = sorted({r for (c, r) in parking_bay_cells})
    min_c, max_c = bay_cols[0], bay_cols[-1]
    min_r, max_r = bay_rows[0], bay_rows[-1]

    # Entrance parameters (in cells): leave gap_width cells free in the flank to act as entrance
    # Choose gap_width = 2 (2*3m = 6m) to be safe for truck; set 1 for smaller vehicles
    gap_width = 2

    # Entrance center row: choose the middle row of the bay (for small bay this will center)
    entrance_center_row = (min_r + max_r) // 2

    # Rows that will remain free for the entrance
    entrance_rows = []
    half_gap = gap_width // 2
    for dr in range(-half_gap, half_gap + (gap_width % 2)):
        r = entrance_center_row + dr
        if 0 <= r < GRID_SIZE_CELLS:
            entrance_rows.append(r)

    # WEST flank (cells immediately to the left of bay)
    left_flank_col = min_c - 1
    if 0 <= left_flank_col < GRID_SIZE_CELLS:
        for r in bay_rows:
            if r in entrance_rows:
                continue  # keep entrance open here
            if (left_flank_col, r) not in parking_bay_cells:
                occupied_cells.add((left_flank_col, r))

    # EAST flank (cells immediately to the right of bay)
    right_flank_col = max_c + 1
    if 0 <= right_flank_col < GRID_SIZE_CELLS:
        for r in bay_rows:
            if r in entrance_rows:
                continue
            if (right_flank_col, r) not in parking_bay_cells:
                occupied_cells.add((right_flank_col, r))

    # Optional: place a cap (north) but leave entrance rows free
    north_cap_row = max_r + 1
    if 0 <= north_cap_row < GRID_SIZE_CELLS:
        for c in bay_cols:
            if (c, north_cap_row) not in parking_bay_cells and north_cap_row not in entrance_rows:
                occupied_cells.add((c, north_cap_row))
    # --- end flanks with entrance gap ---



        # Final layout validation (boundary buffer, no 3/4 2×2 blocks, bay pad, etc.)
        if has_valid_clearance(occupied_cells):
            return occupied_cells, start_clear_cells, parking_bay_cells

    # Fallback: return the last attempt; planner will handle failure gracefully
    return occupied_cells, start_clear_cells, parking_bay_cells



# ========================
# World class
# ========================

class World:
    """Main world container holding grid, obstacles, start pose, and parking goal."""

    def __init__(self,
                 n: int = 12,
                 cell: float = 3.0,
                 density: float = 0.10,
                 seed: int = 7,
                 trailer: bool = False):

        # 0) Generate cells first (gives you occupied_cells & parking_bay_cells)
        occupied_cells, start_clear_cells, parking_bay_cells = \
            generate_random_obstacle_cells(density, seed)

        # 1) Build & store grid + dims (what run_valet.py expects)
        #    (make sure you have `import numpy as np` at the top of the file)
        grid = np.zeros((GRID_SIZE_CELLS, GRID_SIZE_CELLS), dtype=np.uint8)
        for (col, row) in occupied_cells:
            grid[row, col] = 1
        for (col, row) in parking_bay_cells:   # keep bay clear
            grid[row, col] = 0

        self.grid = grid
        self.W = GRID_SIZE_CELLS
        self.H = GRID_SIZE_CELLS

        # 2) Store meters/cell metadata
        self.cell_size_m = GRID_CELL_SIZE_METERS
        self.grid_size_cells = GRID_SIZE_CELLS

        # 3) Convert obstacles to meter-polygons (used by planner/SAT)
        self.obstacle_polygons = convert_cells_to_polygons(occupied_cells)

        # 4) Parking goal in SE bay (meters). Facing WEST into bay is fine.
        self.parking_info = {
            "cells": parking_bay_cells,
            "goal": {
                "pose": (
                    (GRID_SIZE_CELLS - 1.5) * GRID_CELL_SIZE_METERS,  # x
                    (1.5) * GRID_CELL_SIZE_METERS,                    # y  (adjust if your y-axis is flipped)
                    math.pi                                            # yaw (west)
                ),
                "tol_xy": 1.0,
                "tol_yaw": math.radians(10)
            }
        }

        # 5) Start pose at NW corner (meters) — pick the one matching your coord convention:
        # If y increases UP: use (0.5, GRID_SIZE_CELLS-0.5)
        # If y increases DOWN (image-style): use (0.5, 0.5)       
        start_x = 0.5 * GRID_CELL_SIZE_METERS
        start_y = (GRID_SIZE_CELLS - 0.5) * GRID_CELL_SIZE_METERS   # NW: top row center
        self.start_pose = State2D(start_x, start_y, 0.0)


        # 6) Clamp start inside bounds using a conservative footprint
        half_L = 2.6  # ~5.2/2 m
        half_W = 1.0  # ~2.0/2 m
        Wm = GRID_SIZE_CELLS * GRID_CELL_SIZE_METERS
        cx, cy, th = self.start_pose.x, self.start_pose.y, self.start_pose.theta
        req_x = abs(half_L * math.cos(th)) + abs(half_W * math.sin(th))
        req_y = abs(half_L * math.sin(th)) + abs(half_W * math.cos(th))
        cx = max(req_x + 1e-2, min(Wm - req_x - 1e-2, cx))
        cy = max(req_y + 1e-2, min(Wm - req_y - 1e-2, cy))
        self.start_pose.x, self.start_pose.y = cx, cy

        # 7) Point start heading inward (simple heuristic)
        def _pick_inward_heading(x, y, W):
            cands = (0.0, math.pi/2, math.pi, -math.pi/2)  # →, ↑, ←, ↓
            def clearance(th):
                dx, dy = math.cos(th), math.sin(th)
                ts = []
                if dx > 0:   ts.append((W - x)/dx)
                elif dx < 0: ts.append((0 - x)/dx)
                if dy > 0:   ts.append((W - y)/dy)
                elif dy < 0: ts.append((0 - y)/dy)
                return min(abs(t) for t in ts) if ts else 0.0
            return max(cands, key=clearance)

        self.start_pose.theta = _pick_inward_heading(self.start_pose.x, self.start_pose.y, Wm)


    def obstacles_as_polygons(self):
        """Return list of obstacle polygons (for planner)."""
        return self.obstacle_polygons

