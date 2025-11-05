"""
Firetruck agent (defender).
Continuous pose (x,y,theta), responds to burning obstacles, tries to extinguish.
Uses PRM for navigation (will be wired later).
"""

from typing import Optional, Tuple
from src_env import fire_state

class Firetruck:
    def __init__(self, start_pose: Tuple[float, float, float]):
        # pose in meters, heading in radians
        self.x, self.y, self.theta = start_pose

        # which obstacle we are trying to put out
        self.target_obs: Optional[int] = None

        # how long we've been sitting near that obstacle (sec)
        self.spray_timer: float = 0.0

    def _pick_target(self, world):
        """
        Pick one obstacle that is currently burning.
        Simple policy: first burning obstacle we see.
        """
        if self.target_obs is not None:
            return
        for idx, obs in enumerate(world.obstacles):
            if obs.state == world.STATE_BURNING:
                self.target_obs = idx
                break

    def _move_towards_target(self, world, dt: float):
        """
        Placeholder "motion":
        - if we have a target, move in a straight line toward its center.
        This ignores kinematics for now. PRM will replace this.
        """
        if self.target_obs is None:
            return

        cx, cy = world.obstacles[self.target_obs].center_xy()
        dx = cx - self.x
        dy = cy - self.y
        dist = (dx**2 + dy**2) ** 0.5
        if dist < 1e-6:
            return

        # simple capped speed
        v = 5.0  # m/s max speed placeholder (assignment spec says 10 m/s, so this is conservative)
        step = min(v * dt, dist)
        self.x += (dx / dist) * step
        self.y += (dy / dist) * step
        # heading update (not enforcing turn radius yet)
        # in report we'll say PRM enforces nonholonomic feasibility,
        # this is just a first version.
        # theta could be math.atan2(dy, dx) later.

    def _maybe_extinguish(self, world, dt: float, extinguish_radius=10.0, hold_time=5.0):
        """
        If we are close enough to the burning obstacle, stay there and "spray".
        After hold_time seconds, mark it extinguished.
        """
        if self.target_obs is None:
            return False  # nothing happened

        obs = world.obstacles[self.target_obs]
        if obs.state != world.STATE_BURNING:
            return False  # already handled or burned out

        cx, cy = obs.center_xy()
        dx = cx - self.x
        dy = cy - self.y
        dist = (dx**2 + dy**2) ** 0.5

        if dist <= extinguish_radius:
            # we're in range: increment timer
            self.spray_timer += dt
            if self.spray_timer >= hold_time:
                fire_state.extinguish_obstacle(world, self.target_obs)
                self.target_obs = None
                self.spray_timer = 0.0
                return True  # we actually extinguished
        else:
            # moved away -> reset timer
            self.spray_timer = 0.0

        return False

    def step(self, world, t_now: float, dt: float):
        """
        One simulation tick for the firetruck.
        1. choose a burning obstacle if we don't have one
        2. move toward it
        3. try to extinguish it
        Returns True if we extinguished something on this tick.
        """
        self._pick_target(world)
        self._move_towards_target(world, dt)
        did_extinguish = self._maybe_extinguish(world, dt)
        return did_extinguish

