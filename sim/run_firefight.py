#!/usr/bin/env python3
"""
run_firefight.py
Main simulation driver for RBE550 HW5 (Wildfire).

Responsibilities
---------------
- Build a wildfire world (250 m × 250 m, 5 m cells, tetromino-style obstacles).
- Create agents: Wumpus (discrete/grid) and Firetruck (kinematic with PRM).
- Run a fixed-duration discrete-time loop (simulation time, not wall-clock).
- Log events to per-run CSVs, and write summary + timing tables.

Outputs (written to --outdir)
-----------------------------
- run_<seed>.csv      : timeline of events (t, event, x, y, notes)
- summary.csv         : per-run scores
- champion.txt        : winner (“truck”, “wumpus”, or “none”)
- compute_times.csv   : CPU-time breakdown (wumpus / PRM build / PRM queries)
"""

from __future__ import annotations

import argparse
import csv
import os
import time

from src_env.wildfire_world import WildfireWorld
from src_env.fire_state import ignite_obstacle, update_fire
from agents.wumpus import Wumpus
from agents.firetruck import Firetruck
from plan.prm import PRM


# Log a telemetry row every N simulated seconds
TELEMETRY_DT = 10


def _run_single_sim(seed: int, duration: int, wall_time: float = 0.0):
    """
    Run one wildfire simulation for `duration` seconds of simulation time.

    Returns
    -------
    rows : list[list]    -> [t, event, x, y, notes]
    scores : tuple       -> (wumpus_ignited, wumpus_burned, truck_extinguished)
    timings : tuple      -> (t_wumpus_total, t_prm_build, t_prm_query_total)
    """
    # Safe default for telemetry cadence if not defined elsewhere
    try:
        TELEMETRY = TELEMETRY_DT
    except NameError:
        TELEMETRY = 5

    per_step_budget = (wall_time / float(duration)) if wall_time > 0 else 0.0
    rows = [[0, "init", 0.0, 0.0, f"seed={seed}"]]

    # ---- World ----
    world = WildfireWorld(seed=seed)

    # Seed one ignition at t=0 so spreading/physics have a driver
    if getattr(world, "obstacles", None):
        ignite_obstacle(world, 0, t_now=0)
        x0, y0 = world.obstacles[0].center_xy()
        rows.append([0, "ignite", x0, y0, "obs=0 (seed)"])

    # ---- PRM build timing (global roadmap for Firetruck) ----
    t0 = time.perf_counter()
    prm = PRM(world)
    prm.build()
    t_prm_build = time.perf_counter() - t0

    # ---- Agents ----
    # Wumpus spawn (integer grid cell)
    if hasattr(world, "wumpus_start") and world.wumpus_start is not None:
        w_start_rc = world.wumpus_start  # (r,c)
    else:
        w_start_rc = world.random_free_cell() if hasattr(world, "random_free_cell") else (0, 0)
    wumpus = Wumpus(start_ij=(w_start_rc[1], w_start_rc[0]))  # (c,r)

    # Firetruck spawn (continuous pose)
    if hasattr(world, "firetruck_start") and world.firetruck_start is not None:
        truck = Firetruck(start_pose=world.firetruck_start)
    else:
        ft_rc = world.random_free_cell() if hasattr(world, "random_free_cell") else (0, 0)
        tile = getattr(world, "cell_size_m", 5.0)
        truck = Firetruck(start_pose=(ft_rc[1] * tile + tile / 2.0,
                                      ft_rc[0] * tile + tile / 2.0,
                                      0.0))
    # Attach PRM if the agent exposes a field for it (no-op if not used)
    try:
        truck.prm = prm
    except Exception:
        pass

    # ---- Score & timing accumulators ----
    wumpus_ignited = 0
    wumpus_burned = 0
    truck_extinguished = 0

    t_wumpus_total = 0.0
    t_prm_query_total = 0.0
    burned_awarded: set[int] = set()

    # ---- Main loop (discrete time, 1s steps) ----
    for t_now in range(duration):
        tick_start = time.perf_counter()

        # Heartbeat (every 5 minutes of sim time)
        if t_now % 300 == 0:
            print(f"  [seed={seed}] t={t_now}/{duration}")

        # Telemetry cadence
        if (t_now % TELEMETRY) == 0:
            rows.append([t_now, "telemetry", 0.0, 0.0, ""])

        # ---- Wumpus step (A* path + adjacent ignition) ----
        tA = time.perf_counter()
        prev_states = [obs.state for obs in world.obstacles]
        wumpus.step(world, t_now)
        aft_states = [obs.state for obs in world.obstacles]

        for idx, (before, after) in enumerate(zip(prev_states, aft_states)):
            if before == world.STATE_INTACT and after == world.STATE_BURNING:
                wumpus_ignited += 1
                x, y = world.obstacles[idx].center_xy()
                rows.append([t_now, "ignite", x, y, f"obs={idx}"])
        t_wumpus_total += (time.perf_counter() - tA)

        # ---- Firetruck step (may extinguish) ----
        tB = time.perf_counter()
        did_extinguish = truck.step(world, t_now, dt=1.0)
        if did_extinguish:
            target = getattr(truck, "target_obs", None)
            note = f"obs={target}" if target is not None else ""
            rows.append([t_now, "extinguish", getattr(truck, "x", 0.0), getattr(truck, "y", 0.0), note])
            truck_extinguished += 1
        t_prm_query_total += (time.perf_counter() - tB)

        # ---- Fire physics (spread + burnout) ----
        prev_states = aft_states
        update_fire(world, t_now)
        aft_states = [obs.state for obs in world.obstacles]

        for idx, (before, after) in enumerate(zip(prev_states, aft_states)):
            if before == world.STATE_INTACT and after == world.STATE_BURNING:
                wumpus_ignited += 1
                x, y = world.obstacles[idx].center_xy()
                rows.append([t_now, "ignite", x, y, f"obs={idx}"])
            elif (before == world.STATE_BURNING
                  and after == world.STATE_BURNED
                  and idx not in burned_awarded):
                burned_awarded.add(idx)
                wumpus_burned += 1
                x, y = world.obstacles[idx].center_xy()
                rows.append([t_now, "burned", x, y, f"obs={idx}"])

        # Optional pacing to approximate wall_time
        if per_step_budget > 0.0:
            elapsed = time.perf_counter() - tick_start
            delay = per_step_budget - elapsed
            if delay > 0:
                time.sleep(delay)

    return (
        rows,
        (wumpus_ignited, wumpus_burned, truck_extinguished),
        (t_wumpus_total, t_prm_build, t_prm_query_total),
    )



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--duration', type=int, default=3600)
    ap.add_argument('--outdir', type=str, default='results')
    ap.add_argument('--seed_base', type=int, default=1000)
    ap.add_argument(
        '--wall_time', type=float, default=0.0,
        help='If > 0, target wall-clock seconds per run (for live demos).'
    )
    args = ap.parse_args()

    start_all = time.perf_counter()

    os.makedirs(args.outdir, exist_ok=True)

    compute_rows = []
    summary_rows = [['run_id', 'seed',
                     'wumpus_ignited', 'wumpus_burned', 'truck_extinguished',
                     'wumpus_score', 'truck_score']]
    wins = {'wumpus': 0, 'truck': 0}

    for i in range(args.runs):
        seed = args.seed_base + i
        run_id = f'run_{seed}'

        print(f"[INFO] starting {run_id} (duration={args.duration}s)")
        start_wall = time.perf_counter()

        per_run_rows, scores, timing = _run_single_sim(seed, args.duration, args.wall_time)
        
        # append timings to results/compute_times.csv
        ct_csv = os.path.join(args.outdir, "compute_times.csv")
        header = not os.path.exists(ct_csv)
        with open(ct_csv, "a") as f:
            if header:
                f.write("seed,t_wumpus_total,t_prm_build,t_prm_query_total\n")
            f.write(f"{seed},{timing[0]:.6f},{timing[1]:.6f},{timing[2]:.6f}\n")


        elapsed_wall = time.perf_counter() - start_wall
        print(f"[INFO] finished {run_id} in {elapsed_wall:.2f}s")

        wumpus_ignited, wumpus_burned, truck_extinguished = scores
        t_wumpus, t_prm_build, t_prm_queries = timing

        # Scores per rubric
        wumpus_score = wumpus_ignited + wumpus_burned
        truck_score = 2 * truck_extinguished

        # Per-run CSV
        with open(os.path.join(args.outdir, f'{run_id}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'event', 'x', 'y', 'notes'])
            w.writerows(per_run_rows)

        # Summary
        summary_rows.append([
            run_id, seed,
            wumpus_ignited, wumpus_burned, truck_extinguished,
            wumpus_score, truck_score
        ])

        # Timing table
        compute_rows.append([run_id, t_wumpus, t_prm_build, t_prm_queries])

        # Match win
        if truck_score > wumpus_score:
            wins['truck'] += 1
        elif wumpus_score > truck_score:
            wins['wumpus'] += 1

    # Write summary files
    with open(os.path.join(args.outdir, 'summary.csv'), 'w', newline='') as f:
        csv.writer(f).writerows(summary_rows)

    champion = (
        'truck' if wins['truck'] >= 3
        else 'wumpus' if wins['wumpus'] >= 3
        else 'none'
    )
    with open(os.path.join(args.outdir, 'champion.txt'), 'w') as f:
        f.write(champion + '\n')

    with open(os.path.join(args.outdir, 'compute_times.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run_id', 'wumpus_planning_s', 'firetruck_roadmap_s', 'firetruck_queries_s'])
        w.writerows(compute_rows)

    total_elapsed = time.perf_counter() - start_all
    print(f"\n[INFO] Total runtime: {total_elapsed:.2f} seconds wall time.")


if __name__ == '__main__':
    main()

