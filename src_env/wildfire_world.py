"""
wildfire_world.py
Environment model for RBE550 HW5 (Wildfire).

- Field is 250m x 250m.
- Obstacles are clusters of 5m x 5m tiles (tetromino-style).
- Each obstacle has a fire state: intact / burning / extinguished / burned.
- Wumpus and Firetruck start on opposite sides of the map.
"""

import math
import random

# basic map geometry
WORLD_SIZE_M = 250.0
CELL_SIZE_M = 5.0
GRID_SIZE = int(WORLD_SIZE_M / CELL_SIZE_M)  # 50 x 50

STATE_INTACT = "intact"
STATE_BURNING = "burning"
STATE_EXTINGUISHED = "extinguished"
STATE_BURNED = "burned"


class WildfireObstacle:
    def __init__(self, cells):
        """
        cells: set of (col,row) grid coords using 5m cells.
        We'll derive a rough polygon/center from that.
        """
        self.cells = set(cells)
        self.state = STATE_INTACT
        self.burn_start_time = None  # when it started burning (sim time)

    def center_xy(self):
        # average cell center in meters
        if not self.cells:
            return (0.0, 0.0)
        xs = []
        ys = []
        for (c, r) in self.cells:
            x = (c + 0.5) * CELL_SIZE_M
            y = (r + 0.5) * CELL_SIZE_M
            xs.append(x)
            ys.append(y)
        return (sum(xs) / len(xs), sum(ys) / len(ys))


class WildfireWorld:
    def __init__(self, seed=0, density=0.10):
        self.seed = seed
        self.rng = random.Random(seed)

        # expose geometry so plotting doesn't guess
        self.WORLD_SIZE_M = WORLD_SIZE_M
        self.CELL_SIZE_M  = CELL_SIZE_M

        # expose fire state labels for agents
        self.STATE_INTACT = STATE_INTACT
        self.STATE_BURNING = STATE_BURNING
        self.STATE_EXTINGUISHED = STATE_EXTINGUISHED
        self.STATE_BURNED = STATE_BURNED

        # agent spawns
        self.wumpus_start = (GRID_SIZE - 5, GRID_SIZE // 2)   # grid coords (i,j)
        self.firetruck_start = (20.0, WORLD_SIZE_M / 2.0, 0.0)

        self.obstacles = []
        self._generate_obstacles(density)

    def _generate_obstacles(self, density):
        """
        Goal: ~10% coverage of the 50x50 grid using tetromino-style pieces.
        For now this just sprinkles placeholder single-cell obstacles so that
        the rest of the pipeline can run. I can improve the packing logic later.
        """
        total_cells = GRID_SIZE * GRID_SIZE
        target_cells = int(total_cells * density)

        used = set()
        obstacles = []

        # simple fallback: place 1-cell "obstacles" at random non-overlapping cells
        # (later I can group 4 cells into tetromino shapes, like in HW4)
        attempts = 0
        while len(used) < target_cells and attempts < target_cells * 10:
            c = self.rng.randrange(0, GRID_SIZE)
            r = self.rng.randrange(0, GRID_SIZE)
            if (c, r) in used:
                attempts += 1
                continue
            # keep a safety band near the borders open
            if c < 2 or c > GRID_SIZE - 3 or r < 2 or r > GRID_SIZE - 3:
                attempts += 1
                continue

            used.add((c, r))
            obstacles.append(WildfireObstacle({(c, r)}))

        self.obstacles = obstacles

    def distance_between_obstacles(self, i, j):
        xi, yi = self.obstacles[i].center_xy()
        xj, yj = self.obstacles[j].center_xy()
        dx = xi - xj
        dy = yi - yj
        return math.hypot(dx, dy)

