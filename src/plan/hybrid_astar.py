# plan/hybrid_astar.py
import math, heapq
from vehicles.ackermann import Ackermann
from vehicles.base import State2D
from geom.polygons import oriented_box
from geom.collision import first_collision


IGNORE_COLLISIONS_FOR_BRINGUP = False
CLEARANCE_MARGIN_METERS = 0.19  # small gap so planned path doesn't hug obstacles
#SAFETY_MARGIN_M = 0.12   # 20 cm inflation of the vehicle footprint for collision checks
#MAX_EDGE_SAMPLE_SPACING = 0.10  # ≤30 cm between consecutive collision samples

SAFETY_MARGIN_M = 0.19
MAX_EDGE_SAMPLE_SPACING = 0.20
# keep vehicle safely inside the map during rollouts & smoothing
OUT_OF_BOUNDS_MARGIN_M = 0.00   # small tolerance so we don’t reject numerically-equal boundary




def _wrap(a):  # -> [-pi, pi]
    return (a + math.pi) % (2*math.pi) - math.pi


def _get_step_fn(model):
    """
    Return a callable that advances the state one step.
    Works with models exposing .step/.propagate/.integrate/.forward.
    If none exist (e.g., TruckTrailerFollower), falls back to an internal
    Ackermann integrator using the model's wheelbase (model.L).
    """
    # Try the common method names first
    for name in ("step", "propagate", "integrate", "forward"):
        if hasattr(model, name) and callable(getattr(model, name)):
            return getattr(model, name)

    # Fallback: simulate truck front using an Ackermann proxy
    if hasattr(model, "L"):
        inner = Ackermann(
            length=getattr(model, "length", getattr(model, "truck_len", 4.5)),
            width=getattr(model, "width", getattr(model, "truck_w", 2.0)),
            wheelbase=model.L,  # ✅ correct keyword argument
        )

        def step_like(s, v, steer, dt):
            """Fallback step using Ackermann kinematics."""
            return inner.step(s, v, steer, dt)

        return step_like

    # If we reach here, no valid stepping method was found
    raise AttributeError(
        f"{type(model).__name__} has no step-like method "
        "(tried: step, propagate, integrate, forward) and no 'L' for fallback."
    )


def _call_step_flex(step_fn, s, v, steer, dt):
    """Best-effort call; normalize returns to State2D if needed."""
    out = step_fn(s, v, steer, dt)
    if out is None:
        # Defensive: keep previous state if a model returns None
        return State2D(s.x, s.y, s.theta)
    if isinstance(out, State2D):
        return out
    if isinstance(out, tuple) and len(out) >= 3:
        return State2D(out[0], out[1], out[2])
    if isinstance(out, tuple) and len(out) > 0:
        # Some step() impls return (next_state, *extras)
        nxt = out[0]
        if isinstance(nxt, State2D):
            return nxt
        if isinstance(nxt, tuple) and len(nxt) >= 3:
            return State2D(nxt[0], nxt[1], nxt[2])
    # Last resort: pass through but ensure it looks like a State2D
    return State2D(getattr(out, "x", s.x), getattr(out, "y", s.y), getattr(out, "theta", s.theta))


def _rollout(model, s: State2D, v, steer, T=0.3, dt=0.05):
    step_fn = _get_step_fn(model)
    samples = []
    cur = State2D(s.x, s.y, s.theta)
    t = 0.0
    # Hard cap against numerical drift (ensures loop terminates even if T/dt is imperfect)
    max_iters = max(1, int(T / dt) + 3)
    it = 0
    while t <= T + 1e-9 and it < max_iters:
        samples.append((cur.x, cur.y, cur.theta))
        nxt = _call_step_flex(step_fn, cur, v, steer, dt)
        # Defensive: if a model returns NaNs or something invalid, stop this edge early
        if not (math.isfinite(nxt.x) and math.isfinite(nxt.y) and math.isfinite(nxt.theta)):
            break
        cur = nxt
        t += dt
        it += 1
    return samples, cur


def _inside_goal(p: State2D, goal):
    gx, gy, gth = goal["pose"]
    tol_xy, tol_yaw = goal["tol_xy"], goal["tol_yaw"]
    if math.hypot(p.x - gx, p.y - gy) <= tol_xy:
        return abs(_wrap(p.theta - gth)) <= tol_yaw
    return False


def _disc(s: State2D, cell=1.0, dth=math.radians(30)):
    th = _wrap(s.theta)
    return (int(round(s.x / cell)), int(round(s.y / cell)), int(round(th / dth)))


def _heur(x, y, gx, gy, th=None, gth=None):
    h = math.hypot(x - gx, y - gy)
    return h if th is None else h + 0.5 * abs(_wrap(th - gth))  # was 0.25


class HybridAStar:
    def __init__(self, world, car: Ackermann,
                 dt=0.03, step_T=0.2,
                 steer_set=(-0.60, -0.30, 0.0, 0.30, 0.60),
                 speed_set=(-1.0, 1.0),
                 grid_cell=1.0,
                 theta_bin=math.radians(30)):
        self.world = world
        self.car = car
        self.dt = dt
        self.step_T = step_T
        self.steer_set = steer_set
        self.speed_set = speed_set
        self.grid_cell = grid_cell
        self.theta_bin = theta_bin
        self.obstacles = world.obstacles_as_polygons()



    def _edge_collision(self, samples):
        if IGNORE_COLLISIONS_FOR_BRINGUP:
            return False
        if not self.obstacles:
            return False

        # Detect if this is a truck+trailer model (attribute names per your file)
        has_trailer = any(hasattr(self.car, nm) for nm in ("trailer_len", "trailer_w", "d1", "hitch_d"))

        # Vehicle dims (inflated) for the primary body
        L_truck = getattr(self.car, "length", getattr(self.car, "truck_len", 4.5))
        W_truck = getattr(self.car, "width",  getattr(self.car, "truck_w",  2.0))
        L_eff_truck = L_truck + 2*SAFETY_MARGIN_M
        W_eff_truck = W_truck + 2*SAFETY_MARGIN_M

        # Trailer dims (inflated), if present
        if has_trailer:
            L_trailer = getattr(self.car, "trailer_len", 4.5)
            W_trailer = getattr(self.car, "trailer_w",  2.0)
            d_hitch   = getattr(self.car, "d1", getattr(self.car, "hitch_d", 5.0))
            L_eff_trailer = L_trailer + 2*SAFETY_MARGIN_M
            W_eff_trailer = W_trailer + 2*SAFETY_MARGIN_M

        # Densify samples along the edge so we don’t slip between checks
        pts = samples
        dense = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0, _   = pts[i-1]
            x1, y1, th1 = pts[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(seg / MAX_EDGE_SAMPLE_SPACING))
            for k in range(1, n + 1):
                t = k / n
                # linear interp x,y ; theta use last (short step) — good enough for dense samples
                dense.append((x0 + t*(x1 - x0), y0 + t*(y1 - y0), th1))

        # Build footprints and test collision
        for (x, y, th) in dense:
            # Truck rectangle (centered at truck ref pose (x,y,th))
            poly_truck = oriented_box((x, y), L_eff_truck, W_eff_truck, th)

            # Check truck first
            k, _ = first_collision([poly_truck], self.obstacles)
            if k is not None:
                return True

            # Trailer rectangle, if any
            if has_trailer:
                # Trailer reference at axle center, approx aligned with truck heading (low-speed)
                xt = x - d_hitch * math.cos(th)
                yt = y - d_hitch * math.sin(th)
                th_trailer = th  # simplest robust approximation; your model may provide a better one
                poly_trailer = oriented_box((xt, yt), L_eff_trailer, W_eff_trailer, th_trailer)
                k2, _ = first_collision([poly_trailer], self.obstacles)
                if k2 is not None:
                    return True

        return False



    def _rollout_out_of_bounds(self, samples):
        """
        True if any inflated vehicle footprint vertex goes outside [0,W]x[0,H].
        We skip the very first sample (the current node) so a start placed close to a
        boundary may take a step that moves it back into a safe region.
        """
        W = self.world.grid_size_cells * self.world.cell_size_m
        H = W  # square world
        eps = OUT_OF_BOUNDS_MARGIN_M

        # same dims as collision checks
        L = getattr(self.car, "length", getattr(self.car, "truck_len", 4.5))
        Wd = getattr(self.car, "width",  getattr(self.car, "truck_w",  2.0))
        L_eff = L + 2*SAFETY_MARGIN_M
        W_eff = Wd + 2*SAFETY_MARGIN_M

        # SKIP the first sample (it equals the current state)
        for (x, y, th) in samples[1:]:
            box = oriented_box((x, y), L_eff, W_eff, th)
            for (px, py) in box:
                if px < -eps or py < -eps or px > W + eps or py > H + eps:
                    return True
        return False


    def plan(self, start_pose, goal, iters_limit=200000):
        start = State2D(start_pose.x, start_pose.y, start_pose.theta)
        if _inside_goal(start, goal):
            return [(start.x, start.y, start.theta)]

        gx, gy, gth = goal["pose"]
        key0 = _disc(start, self.grid_cell, self.theta_bin)
        g_cost = {key0: 0.0}
        openpq = [( _heur(start.x, start.y, gx, gy, start.theta, gth), 0.0, start, key0 )]
        parent = {}
        visited = set()
        it = 0

        while openpq and it < iters_limit:
            it += 1
            self.expanded = it 
            f, g, s, skey = heapq.heappop(openpq)
            if it % 2000 == 0:
                print(f"expanded={it}, open={len(openpq)}")
            if skey in visited:
                continue
            visited.add(skey)

            if _inside_goal(s, goal):
                path = []
                k = skey; cur = s
                while k in parent:
                    path.append((cur.x, cur.y, cur.theta))
                    k, cur = parent[k]
                path.append((start.x, start.y, start.theta))
                path.reverse()
                return path

            for v in self.speed_set:
                for steer in self.steer_set:
                    samples, s2 = _rollout(self.car, s, v, steer, self.step_T, self.dt)

                    if self._edge_collision(samples):
                        continue
                    #if self._rollout_out_of_bounds(samples):
                    #    continue

                    k2 = _disc(s2, self.grid_cell, self.theta_bin)
                    newg = g + self.step_T * abs(v)
                    if (k2 not in g_cost) or (newg < g_cost[k2]):
                        g_cost[k2] = newg
                        parent[k2] = (skey, s)
                        h = _heur(s2.x, s2.y, gx, gy, s2.theta, gth)
                        heapq.heappush(openpq, (newg + h, newg, s2, k2))

        return None

