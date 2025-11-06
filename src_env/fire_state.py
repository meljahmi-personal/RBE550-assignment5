# src_env/fire_state.py
# Fire model that matches wildfire_world.py (states are STRINGS)

STATE_INTACT       = "intact"
STATE_BURNING      = "burning"
STATE_EXTINGUISHED = "extinguished"
STATE_BURNED       = "burned"

# Physics (per HW spec)
SPREAD_DELAY_S  = 10        # after 10 s of burning, ignite neighbors
SPREAD_RADIUS_M = 30.0      # meters (center-to-center)
BURNOUT_TOTAL_S = 25        # burn for 25 s total, then become "burned"

def ignite_obstacle(world, idx: int, t_now: int) -> None:
    """Set obstacle idx to BURNING (only from INTACT)."""
    obs = world.obstacles[idx]
    if obs.state == STATE_INTACT:
        obs.state = STATE_BURNING
        obs.burn_start_time = t_now
        
        

def try_extinguish(world, idx: int, t_now: int | None = None) -> bool:
    """Set obstacle idx to EXTINGUISHED (only from BURNING). Returns True if changed."""
    obs = world.obstacles[idx]
    if obs.state == STATE_BURNING:
        obs.state = STATE_EXTINGUISHED
        obs.burn_start_time = None
        return True
    return False


def extinguish_obstacle(world, idx: int, t_now: int | None = None) -> bool:
    """Compatibility wrapper used by Firetruck.step()."""
    return try_extinguish(world, idx, t_now)

def update_fire(world, t_now: int) -> None:
    """
    1) At exactly SPREAD_DELAY_S after ignition, burning obstacles ignite
       any INTACT neighbors within SPREAD_RADIUS_M.
    2) After BURNOUT_TOTAL_S, BURNING obstacles become BURNED.
    """
    # 1) spread from "just hit the spread moment" burners
    spreaders = []
    for i, obs in enumerate(world.obstacles):
        if obs.state == STATE_BURNING and obs.burn_start_time is not None:
            if (t_now - obs.burn_start_time) == SPREAD_DELAY_S:
                spreaders.append(i)

    for i in spreaders:
        xi, yi = world.obstacles[i].center_xy()
        for j, other in enumerate(world.obstacles):
            if other.state != STATE_INTACT:
                continue
            xj, yj = other.center_xy()
            dx, dy = xi - xj, yi - yj
            if (dx*dx + dy*dy) ** 0.5 <= SPREAD_RADIUS_M:
                ignite_obstacle(world, j, t_now)

    # 2) burn out
    for obs in world.obstacles:
        if obs.state == STATE_BURNING and obs.burn_start_time is not None:
            if (t_now - obs.burn_start_time) >= BURNOUT_TOTAL_S:
                obs.state = STATE_BURNED
                obs.burn_start_time = None

