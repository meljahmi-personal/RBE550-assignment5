"""
PRM planner skeleton for the Firetruck.

This class is intentionally lightweight right now:
- build() pretends to construct a roadmap
- query() pretends to return a path
We just need it to exist so the simulation driver runs end-to-end.
"""

import math
import random
from typing import List, Tuple

class PRM:
    def __init__(self, world, n_samples: int = 200, k_neighbors: int = 10, seed: int = 0):
        """
        world: the WildfireWorld instance (obstacles, bounds, etc)
        n_samples: how many milestones to try to sample
        k_neighbors: how many nearby edges to attempt per milestone
        seed: deterministic sampling
        """
        self.world = world
        self.n_samples = n_samples
        self.k_neighbors = k_neighbors
        self.rng = random.Random(seed)

        # These will be populated by build()
        self.nodes: List[Tuple[float, float]] = []  # [(x,y), ...]
        self.edges: List[Tuple[int, int, float]] = []  # [(i,j,dist), ...]

    def _collision_free(self, x: float, y: float) -> bool:
        """
        Very cheap placeholder collision check:
        returns False if point is 'inside' any obstacle's bounding circle,
        True otherwise.
        You will replace this later with polygon vs radius checks.
        """
        for obs in self.world.obstacles:
            cx, cy = obs.center_xy()
            dx = x - cx
            dy = y - cy
            d = math.hypot(dx, dy)

            # crude "obstacle radius": half cell diagonal of 5m cell ~ 3.6m
            # inflate a bit
            if d < 4.0:
                return False
        return True

    def build(self):
        """
        Sample n_samples free points and connect each to k_neighbors nearest.
        This is a dumb PRM but it's enough for timing, for structure,
        and for passing into the Firetruck.
        """
        # 1) sample milestones
        W = self.world.WORLD_SIZE_M
        for _ in range(self.n_samples):
            for _try in range(100):  # give up after 100 failed collision samples
                x = self.rng.uniform(0.0, W)
                y = self.rng.uniform(0.0, W)
                if self._collision_free(x, y):
                    self.nodes.append((x, y))
                    break

        # 2) connect neighbors (fully naive O(n^2), no kdtree, simple)
        for i, (xi, yi) in enumerate(self.nodes):
            # compute distances to all others
            dlist = []
            for j, (xj, yj) in enumerate(self.nodes):
                if j == i:
                    continue
                dij = math.hypot(xi - xj, yi - yj)
                dlist.append((dij, j))
            # sort by distance
            dlist.sort(key=lambda pair: pair[0])
            # take k_neighbors closest and add edges
            for d, j in dlist[: self.k_neighbors]:
                self.edges.append((i, j, d))

        # At this point we have a crude roadmap graph.
        # No return. The object just keeps nodes/edges.

    def query(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> List[Tuple[float,float]]:
        """
        Placeholder path lookup.
        For now:
        - snap start and goal to nearest roadmap nodes,
        - then return [start_xy, goal_xy] directly (straight line).
        Later, you'd run Dijkstra or A* over self.edges.
        """
        # Snap start and goal to nearest milestones (not actually used yet downstream,
        # but we keep the API).
        def nearest_node(pt):
            (px, py) = pt
            best = None
            best_d = float('inf')
            for idx, (nx, ny) in enumerate(self.nodes):
                dd = math.hypot(px - nx, py - ny)
                if dd < best_d:
                    best_d = dd
                    best = idx
            return best

        _s_idx = nearest_node(start_xy)
        _g_idx = nearest_node(goal_xy)

        # Return trivial straight-line path for now.
        return [start_xy, goal_xy]

