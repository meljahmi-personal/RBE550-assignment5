"""
Wumpus agent (attacker).
Discrete grid motion; plans with A* on an occupancy grid and ignites obstacles
when standing on a free cell that is 4-connected adjacent to a target obstacle cell.
"""

from typing import List, Tuple, Optional
import numpy as np

# A* facade (row,col API) and 4-neighbor generator
from plan.astar import plan as astar_plan
from plan.neighbors import neighbors4

from src_env import fire_state

# Fire-state codes (kept local for clarity; world also exposes these)
STATE_INTACT = 0
STATE_BURNING = 1
STATE_EXTINGUISHED = 2
STATE_BURNED = 3


class Wumpus:
    """
    Maintains a grid position (integer cell indices), a current obstacle target,
    and a path (list of grid cells). Each simulation step:
      1) picks a target obstacle if none is set,
      2) plans with A* from current cell to a free “vantage” cell adjacent to
         that obstacle (if no current path),
      3) advances one grid cell along the path,
      4) ignites the obstacle when adjacent (4-neighborhood).
    """

    def __init__(self, start_ij: Tuple[int, int]):
        # Grid coordinate (col, row)
        self.ij = start_ij  # (c, r)

        # Index of the obstacle intended for ignition
        self.target_obs: Optional[int] = None

        # Planned grid path as [(c, r), ...]
        self.path: List[Tuple[int, int]] = []

    # --------------------------------------------------------------------- #
    #                         SIMULATION STEP LOOP                          #
    # --------------------------------------------------------------------- #

    def step(self, world, t_now: float):
        """
        One simulation tick:
          - pick target if none
          - plan path if none
          - move one grid cell along current path
          - ignite when adjacent to the target obstacle
        """
        self._maybe_pick_target(world)
        if self.target_obs is None:
            return  # no intact obstacle available

        if not self.path:
            self._plan_path_to_obstacle(world)

        if self.path:
            # Paths are stored as [(c, r), ...]; pop the next waypoint
            self.ij = self.path.pop(0)

        if self.target_obs is not None:
            if self._is_adjacent_to_obstacle(self.ij, world.obstacles[self.target_obs]):
                fire_state.ignite_obstacle(world, self.target_obs, t_now)
                self.target_obs = None
                self.path.clear()

    # --------------------------------------------------------------------- #
    #                           TARGET SELECTION                            #
    # --------------------------------------------------------------------- #

    def _maybe_pick_target(self, world) -> None:
        """
        Picks one intact obstacle as the current target if none is set.
        Selection policy: nearest intact obstacle by grid distance (center-to-center).
        """
        if self.target_obs is not None:
            return

        intact_indices = [
            idx for idx, obs in enumerate(world.obstacles)
            if getattr(obs, "state", None) == getattr(world, "STATE_INTACT", STATE_INTACT)
        ]
        if not intact_indices:
            return

        c0, r0 = self.ij

        def obs_center(idx):
            cells = world.obstacles[idx].cells  # iterable of (c, r)
            cs = [c for (c, _) in cells]
            rs = [r for (_, r) in cells]
            return (sum(cs) / len(cs), sum(rs) / len(rs))

        self.target_obs = min(
            intact_indices,
            key=lambda k: (obs_center(k)[0] - c0) ** 2 + (obs_center(k)[1] - r0) ** 2
        )

    # --------------------------------------------------------------------- #
    #                           PATH PLANNING (A*)                          #
    # --------------------------------------------------------------------- #

    def _plan_path_to_obstacle(self, world) -> None:
        """
        Plans an A* path on an occupancy grid from the current cell to a free
        vantage cell that is 4-connected adjacent to any cell of the target obstacle.
        The A* interface expects (row, col) indexing, so conversions are handled.
        """
        if self.target_obs is None:
            return

        obst_rc = self._build_occupancy_rc(world)
        vantage_rc_list = self._adjacent_free_vantage_rc(world, obst_rc, self.target_obs)
        if not vantage_rc_list:
            self.target_obs = None
            return

        start_rc = (self.ij[1], self.ij[0])  # (r,c)

        def rc_l2(rc):
            return (rc[0] - start_rc[0]) ** 2 + (rc[1] - start_rc[1]) ** 2

        goal_rc = min(vantage_rc_list, key=rc_l2)

        path_rc, _stats = astar_plan(
            grid_rc=obst_rc,
            start_rc=start_rc,
            goal_rc=goal_rc,
            neighbors_fn=neighbors4,
            heuristic="euclidean",
        )

        self.path = [(c, r) for (r, c) in path_rc] if path_rc else []

    # --------------------------------------------------------------------- #
    #                        OCCUPANCY & VANTAGE SETS                       #
    # --------------------------------------------------------------------- #

    def _build_occupancy_rc(self, world) -> np.ndarray:
        """
        Constructs a binary occupancy array indexed by (row, col):
          - 1 indicates blocked (any obstacle cell),
          - 0 indicates free.
        """
        shape = getattr(world, "grid_shape", None)
        if shape is None:
            max_c = 0
            max_r = 0
            for obs in world.obstacles:
                for (c, r) in obs.cells:
                    max_c = max(max_c, c)
                    max_r = max(max_r, r)
            shape = (max_r + 1, max_c + 1)

        obst_rc = np.zeros(shape, dtype=np.uint8)
        for obs in world.obstacles:
            for (c, r) in obs.cells:
                if 0 <= r < obst_rc.shape[0] and 0 <= c < obst_rc.shape[1]:
                    obst_rc[r, c] = 1
        return obst_rc

    def _adjacent_free_vantage_rc(self, world, obst_rc: np.ndarray, obs_idx: int):
        """
        Returns a list of (row, col) cells that are free (0 in obst_rc) and lie
        in the 4-neighborhood of any cell of the obstacle with index obs_idx.
        """
        H, W = obst_rc.shape
        out = []
        tgt = world.obstacles[obs_idx]
        for (c, r) in tgt.cells:
            for (nr, nc) in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < H and 0 <= nc < W and obst_rc[nr, nc] == 0:
                    out.append((nr, nc))
        return list(dict.fromkeys(out))

    # --------------------------------------------------------------------- #
    #                               HELPERS                                 #
    # --------------------------------------------------------------------- #

    @staticmethod
    def _is_adjacent_to_obstacle(ij: Tuple[int, int], obstacle) -> bool:
        """
        Returns True if (c, r) is 4-connected adjacent to any (c_obs, r_obs) in obstacle.cells.
        """
        c, r = ij
        for (oc, orow) in obstacle.cells:
            if abs(c - oc) + abs(r - orow) == 1:
                return True
        return False

