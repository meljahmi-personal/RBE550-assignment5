# sim/run_firefight.py
#!/usr/bin/env python3
import argparse, csv, os

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--duration', type=int, default=3600)
    ap.add_argument('--outdir', type=str, default='results')
    ap.add_argument('--seed_base', type=int, default=1000)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # placeholders until real sim is wired
    compute_rows = []  # run_id,wumpus_planning_s,firetruck_roadmap_s,firetruck_queries_s
    summary_rows = [['run_id','seed','wumpus_ignited','wumpus_burned','truck_extinguished','wumpus_score','truck_score']]
    wins = {'wumpus':0, 'truck':0}

    for i in range(args.runs):
        seed = args.seed_base + i
        run_id = f'run_{seed}'

        # per-run events CSV (placeholder)
        with open(os.path.join(args.outdir, f'{run_id}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','event','x','y','notes'])
            w.writerow([0, 'init', 0.0, 0.0, 'placeholder'])

        # placeholder scores (all zeros for now)
        wumpus_ignited = 0
        wumpus_burned = 0
        truck_extinguished = 0
        wumpus_score = wumpus_ignited + wumpus_burned
        truck_score = 2 * truck_extinguished
        summary_rows.append([run_id, seed, wumpus_ignited, wumpus_burned, truck_extinguished, wumpus_score, truck_score])

        # placeholder timings (fill with real numbers later)
        t_wumpus = 0.0
        t_prm_build = 0.0
        t_prm_queries = 0.0
        compute_rows.append([run_id, t_wumpus, t_prm_build, t_prm_queries])

        # update wins (tie -> none)
        if truck_score > wumpus_score: wins['truck'] += 1
        elif wumpus_score > truck_score: wins['wumpus'] += 1

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
        w.writerows(compute_rows)

if __name__ == '__main__':
    main()

