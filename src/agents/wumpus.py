"""
Wumpus agent (attacker).
Moves on a grid, plans with A*, and ignites obstacles.
"""

from typing import List, Tuple, Optional
from src_env import fire_state

class Wumpus:
    def __init__(self, start_ij: Tuple[int, int]):
        # grid coords (i,j) in 5m cells
        self.ij = start_ij

        # which obstacle we're trying to ignite
        self.target_obs: Optional[int] = None

        # planned path as list of grid cells [(i,j), ...]
        self.path: List[Tuple[int,int]] = []

    def _plan_path_to_obstacle(self, world) -> None:
        """
        Plan a path (grid A*) from self.ij to a cell next to the target obstacle.
        Fills self.path.
        """
        # TODO: build occupancy grid from world.obstacles
        # TODO: run A* and store result in self.path
        return

    def _maybe_pick_target(self, world) -> None:
        """
        If we don't have a target, choose one intact obstacle.
        Simple policy: first intact we find.
        """
        if self.target_obs is not None:
            return
        for idx, obs in enumerate(world.obstacles):
            if obs.state == world.STATE_INTACT:
                self.target_obs = idx
                break

    def step(self, world, t_now: float):
        """
        One simulation tick:
        - choose target if needed
        - plan if we have no path
        - move one grid step along the path
        - if adjacent to target obstacle, ignite it
        """
        self._maybe_pick_target(world)

        if self.target_obs is None:
            return  # nothing to do

        if not self.path:
            self._plan_path_to_obstacle(world)

        # move one step if we actually have a path
        if self.path:
            self.ij = self.path.pop(0)

        # check adjacency to target obstacle cells
        if self.target_obs is not None:
            obs = world.obstacles[self.target_obs]
            for (oc, orow) in obs.cells:
                if abs(self.ij[0] - oc) <= 1 and abs(self.ij[1] - orow) <= 1:
                    fire_state.ignite_obstacle(world, self.target_obs, t_now)
                    break

