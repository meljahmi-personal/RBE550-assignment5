# vehicles/firetruck.py
import math

class Firetruck:
    def __init__(self):
        # HW5 Unimog parameters (RBE550 Wildfire)
        self.width_m = 2.2
        self.length_m = 4.9
        self.wheelbase_m = 3.0
        self.r_min_m = 13.0
        self.v_max = 10.0  # m/s

        # Derived steering limit from R_min = L / tan(phi_max)  => phi_max = atan(L / R_min)
        self.phi_max = math.atan(self.wheelbase_m / self.r_min_m)

    # Bicycle model step: returns (x, y, theta)
    def step(self, x, y, theta, v, phi, dt):
        # clamp controls
        v = max(-self.v_max, min(self.v_max, v))
        phi = max(-self.phi_max, min(self.phi_max, phi))

        if abs(phi) < 1e-6:
            # straight
            nx = x + v * math.cos(theta) * dt
            ny = y + v * math.sin(theta) * dt
            nth = theta
        else:
            # curved
            beta = v / self.wheelbase_m * math.tan(phi)
            nx = x + v * math.cos(theta) * dt
            ny = y + v * math.sin(theta) * dt
            nth = theta + beta * dt
        # normalize heading to [-pi, pi)
        nth = (nth + math.pi) % (2 * math.pi) - math.pi
        return nx, ny, nth

    # Convenience: simulate a short rollout with constant v, phi
    def rollout(self, x, y, theta, v, phi, dt, steps):
        poses = [(x, y, theta)]
        for _ in range(steps):
            x, y, theta = self.step(x, y, theta, v, phi, dt)
            poses.append((x, y, theta))
        return poses

    # Oriented rectangle footprint corners for SAT/collision (cx,cy is center; theta heading)
    def footprint(self, cx, cy, theta, safety_margin=0.0):
        L = self.length_m + 2 * safety_margin
        W = self.width_m  + 2 * safety_margin
        hx, hy = L * 0.5, W * 0.5
        # local corners, centered at vehicle center (front is +x in local frame)
        corners_local = [( hx,  hy), ( hx, -hy), (-hx, -hy), (-hx,  hy)]
        c, s = math.cos(theta), math.sin(theta)
        return [(cx + c*px - s*py, cy + s*px + c*py) for (px, py) in corners_local]

    # Steering helpers
    def steering_limit_rad(self):  # max |phi|
        return self.phi_max

    def speed_limit(self):         # max |v|
        return self.v_max

