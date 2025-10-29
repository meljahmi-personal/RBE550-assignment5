
import math
from vehicles.base import State2D

class Ackermann:
    def __init__(self, length=5.2, width=1.8, wheelbase=2.8):
        self.length = length
        self.width  = width
        self.L = wheelbase   # distance between front and rear axle

    def step(self, state: State2D, v: float, steer: float, dt: float) -> State2D:
        """
        Integrate one step of bicycle kinematics.
        state: current (x, y, theta)
        v:     forward velocity [m/s]
        steer: steering angle   [rad]
        dt:    timestep         [s]
        """
        nx = state.x + v * math.cos(state.theta) * dt
        ny = state.y + v * math.sin(state.theta) * dt
        nth = state.theta + (v / self.L) * math.tan(steer) * dt
        return State2D(nx, ny, nth)




