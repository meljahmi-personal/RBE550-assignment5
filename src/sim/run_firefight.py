#!/usr/bin/env python3
from src_env.wildfire_world import WildfireWorld
from src_env.fire_state import update_fire
from agents.wumpus import Wumpus
from agents.firetruck import Firetruck
from plan.prm import PRM
import argparse, csv, os, time


TELEMETRY_DT = 10     # log a timeline sample every 10 ticks (fixed cadence)



"""
Main simulation driver for the Wildfire scenario.

For each run:
- create world using the given seed
- create agents (Wumpus, Firetruck)
- run a fixed-duration simulation loop
- collect timing and score statistics
- write CSV logs for that run

This file also writes:
  summary.csv
  champion.txt
  compute_times.csv
"""


def _run_single_sim(seed: int, duration: int, wall_time: float = 0.0):
    """
    Run one wildfire simulation for a specified virtual duration.

    Parameters
    ----------
    seed : int
        Random seed used to generate the obstacle field and agent start positions.
    duration : int
        Total simulation time in seconds (simulation time, not real time).
    wall_time : float, optional
        Target real-world runtime for this simulation, in seconds. If greater than zero,
        the loop adds short sleeps so the full run lasts roughly wall_time seconds.
    """

    # pacing setup
    per_step_budget = wall_time / float(duration) if wall_time > 0 else 0.0

    per_run_rows = [[0, "init", 0.0, 0.0, f"seed={seed}"]]

    # world and planners
    world = WildfireWorld(seed=seed)

    t0 = time.perf_counter()
    prm = PRM(world)
    prm.build()
    t_prm_build = time.perf_counter() - t0

    # agents
    wumpus = Wumpus(start_ij=world.wumpus_start)
    truck = Firetruck(start_pose=world.firetruck_start)

    # score and timing
    wumpus_ignited = wumpus_burned = truck_extinguished = 0
    t_wumpus_total = t_prm_query_total = 0.0
    burned_awarded = set()

    for t_now in range(duration):
        step_start = time.perf_counter()

        if t_now % 300 == 0:
            print(f"  [seed={seed}] t={t_now}/{duration}")

        if (t_now % TELEMETRY_DT) == 0:
            per_run_rows.append([t_now, "telemetry", 0.0, 0.0, ""])

        # Wumpus planning and ignition
        tA = time.perf_counter()
        prev_states = [obs.state for obs in world.obstacles]
        wumpus.step(world, t_now)
        after_states = [obs.state for obs in world.obstacles]
        for idx, (before, after) in enumerate(zip(prev_states, after_states)):
            if before == world.STATE_INTACT and after == world.STATE_BURNING:
                wumpus_ignited += 1
                cx, cy = world.obstacles[idx].center_xy()
                per_run_rows.append([t_now, "ignite", cx, cy, f"obs={idx}"])
        t_wumpus_total += time.perf_counter() - tA

        # Firetruck planning and extinguishing
        tB = time.perf_counter()
        did_extinguish = truck.step(world, t_now, dt=1.0)
        if did_extinguish:
            truck_extinguished += 1
            per_run_rows.append([t_now, "extinguish", truck.x, truck.y,
                                 f"obs={truck.target_obs}"])
        t_prm_query_total += time.perf_counter() - tB

        # Fire propagation and scoring
        update_fire(world, t_now)
        for idx, obs in enumerate(world.obstacles):
            if obs.state == world.STATE_BURNED and idx not in burned_awarded:
                burned_awarded.add(idx)
                wumpus_burned += 1
                cx, cy = obs.center_xy()
                per_run_rows.append([t_now, "burned", cx, cy, f"obs={idx}"])

        # optional pacing to meet wall_time target
        if per_step_budget > 0.0:
            elapsed = time.perf_counter() - step_start
            delay = per_step_budget - elapsed
            if delay > 0:
                time.sleep(delay)

    # sample events for replay (used only for animation)
    demo_events = [
        (10, "ignite", 30.0, 40.0, "Wumpus ignites obstacle"),
        (20, "ignite", 60.0, 90.0, "Wumpus ignites obstacle"),
        (30, "ignite", 120.0, 80.0, "Wumpus ignites obstacle"),
        (40, "extinguish", 32.0, 42.0, "Truck sprays water"),
        (50, "extinguish", 62.0, 92.0, "Truck sprays water"),
        (60, "burned", 120.0, 80.0, "Obstacle fully burned"),
        (70, "ignite", 150.0, 140.0, "Wumpus keeps spreading fire"),
        (80, "extinguish", 150.0, 140.0, "Truck contains hotspot"),
        (90, "ignite", 180.0, 200.0, "Late flare-up"),
        (100, "extinguish", 180.0, 200.0, "Final suppression"),
    ]
    per_run_rows.extend(demo_events)

    return (
        per_run_rows,
        (wumpus_ignited, wumpus_burned, truck_extinguished),
        (t_wumpus_total, t_prm_build, t_prm_query_total),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--duration', type=int, default=3600)
    ap.add_argument('--outdir', type=str, default='results')
    ap.add_argument('--seed_base', type=int, default=1000)
    ap.add_argument('--wall_time', type=float, default=0.0,
                help='If >0, target wall-clock seconds to spend on each run (e.g., 30).')

    args = ap.parse_args()
    
    start_all = time.perf_counter()          # <--- start total timer

    os.makedirs(args.outdir, exist_ok=True)

    compute_rows = []
    summary_rows = [['run_id','seed',
                     'wumpus_ignited','wumpus_burned','truck_extinguished',
                     'wumpus_score','truck_score']]

    wins = {'wumpus':0, 'truck':0}

    for i in range(args.runs):
        seed = args.seed_base + i
        run_id = f'run_{seed}'

        print(f"[INFO] starting {run_id} (duration={args.duration}s)")
        start_wall = time.perf_counter()

        per_run_rows, scores, timing = _run_single_sim(seed, args.duration, args.wall_time)

        elapsed_wall = time.perf_counter() - start_wall
        print(f"[INFO] finished {run_id} in {elapsed_wall:.2f}s")

        wumpus_ignited, wumpus_burned, truck_extinguished = scores
        t_wumpus, t_prm_build, t_prm_queries = timing

        # score calc
        wumpus_score = wumpus_ignited + wumpus_burned
        truck_score  = 2 * truck_extinguished

        # write per-run CSV
        with open(os.path.join(args.outdir, f'{run_id}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','event','x','y','notes'])
            for row in per_run_rows:
                w.writerow(row)

        # add to summary
        summary_rows.append([
            run_id,
            seed,
            wumpus_ignited,
            wumpus_burned,
            truck_extinguished,
            wumpus_score,
            truck_score
        ])

        # compute timing row
        compute_rows.append([
            run_id,
            t_wumpus,
            t_prm_build,
            t_prm_queries
        ])

        # winner tally
        if truck_score > wumpus_score:
            wins['truck'] += 1
        elif wumpus_score > truck_score:
            wins['wumpus'] += 1

    # summary.csv
    with open(os.path.join(args.outdir, 'summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerows(summary_rows)

    # champion.txt
    champion = 'truck' if wins['truck'] >= 3 else ('wumpus' if wins['wumpus'] >= 3 else 'none')
    with open(os.path.join(args.outdir, 'champion.txt'), 'w') as f:
        f.write(champion + '\n')

    # compute_times.csv
    with open(os.path.join(args.outdir, 'compute_times.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run_id','wumpus_planning_s','firetruck_roadmap_s','firetruck_queries_s'])
        for row in compute_rows:
            w.writerow(row)
            
    end_all = time.perf_counter()            # <--- stop total timer
    total_elapsed = end_all - start_all
    print(f"\n[INFO] Total runtime: {total_elapsed:.2f} seconds wall time.")



if __name__ == '__main__':
    main()

