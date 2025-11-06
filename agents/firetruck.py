"""
Firetruck agent (defender).
State: (x, y, theta) in meters/radians.
Behavior: pick a burning obstacle, move toward it, extinguish within radius after hold time.
"""

from typing import Optional, Tuple
import math
from src_env import fire_state


class Firetruck:
    def __init__(self, start_pose: Tuple[float, float, float]):
        self.x, self.y, self.theta = start_pose

        # Active target (index into world.obstacles)
        self.target_obs: Optional[int] = None

        # Seconds stayed within extinguish radius of current target
        self.spray_timer: float = 0.0

        # Internal flag to defer clearing target until next tick (so logging sees it)
        self._defer_clear_target: bool = False

        # Simple kinematic placeholders (spec max v = 10 m/s)
        self.v_max = 10.0  # m/s

    # ------------------------
    # Target selection/motion
    # ------------------------
    def _pick_target(self, world) -> None:
        """Pick a burning obstacle. Preference: nearest burning."""
        # If we just extinguished on the previous tick, clear after logging
        if self._defer_clear_target:
            self.target_obs = None
            self._defer_clear_target = False

        if self.target_obs is not None:
            # Keep current target if it’s still burning
            obs = world.obstacles[self.target_obs]
            if obs.state == world.STATE_BURNING:
                return
            # Otherwise, release it and pick anew
            self.target_obs = None

        # Nearest burning obstacle
        best_idx = None
        best_d2 = float("inf")
        for idx, obs in enumerate(world.obstacles):
            if obs.state != world.STATE_BURNING:
                continue
            cx, cy = obs.center_xy()
            dx, dy = (cx - self.x), (cy - self.y)
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_idx = idx

        if best_idx is not None:
            self.target_obs = best_idx

    def _move_towards_target(self, world, dt: float) -> None:
        """Translate toward target center with capped speed (straight-line placeholder)."""
        if self.target_obs is None:
            return

        cx, cy = world.obstacles[self.target_obs].center_xy()
        dx, dy = (cx - self.x), (cy - self.y)
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return

        step = min(self.v_max * dt, dist)
        ux, uy = (dx / dist), (dy / dist)

        self.x += ux * step
        self.y += uy * step
        # Face direction of travel (placeholder; PRM/local planner will replace)
        self.theta = math.atan2(uy, ux)

    # ------------------------
    # Extinguishing
    # ------------------------
    def _maybe_extinguish(
        self,
        world,
        dt: float,
        extinguish_radius: float = 10.0,
        hold_time: float = 5.0,
    ) -> bool:
        """
        If within 'extinguish_radius' of a burning target for >= 'hold_time' seconds,
        mark it extinguished and return True.
        """
        if self.target_obs is None:
            self.spray_timer = 0.0
            return False

        obs = world.obstacles[self.target_obs]
        if obs.state != world.STATE_BURNING:
            self.spray_timer = 0.0
            return False

        cx, cy = obs.center_xy()
        dist = math.hypot(cx - self.x, cy - self.y)

        if dist <= extinguish_radius:
            self.spray_timer += dt
            if self.spray_timer >= hold_time:
                # Preserve target index for the caller’s logging this tick
                idx = self.target_obs
                fire_state.extinguish_obstacle(world, idx)
                # Defer clearing target until the next tick so logging sees obs=<idx>
                self._defer_clear_target = True
                self.spray_timer = 0.0
                return True
        else:
            self.spray_timer = 0.0

        return False

    # ------------------------
    # External interface
    # ------------------------
    def step(self, world, t_now: float, dt: float) -> bool:
        """
        One simulation tick: pick target, move, attempt to extinguish.
        Returns True if an obstacle was extinguished on this tick.
        """
        self._pick_target(world)
        self._move_towards_target(world, dt)
        return self._maybe_extinguish(world, dt)

