# utils/metrics.py
#!/usr/bin/env python3
import argparse, csv, math
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('--out', default='results/compute_times.png')
    args = ap.parse_args()

    sums = [0.0, 0.0, 0.0]  # [wumpus, prm_build, prm_queries]

    with open(args.csv_path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            sums[0] += float(row['wumpus_planning_s'])
            sums[1] += float(row['firetruck_roadmap_s'])
            sums[2] += float(row['firetruck_queries_s'])

    labels = ['Wumpus planning', 'PRM build', 'PRM queries']

    # --- avoid totally blank plot when everything is zero ---
    max_val = max(sums)
    if max_val <= 0.0:
        # put a tiny floor just for visualization, doesn't overwrite data
        display_vals = [1e-6, 1e-6, 1e-6]
    else:
        display_vals = sums

    plt.figure()
    plt.bar(labels, display_vals)
    plt.ylabel('CPU time (s)')
    plt.title('Compute time (sum over runs)')
    # nice y-limit if everything was 0
    if max_val <= 0.0:
        plt.ylim(0, 1e-6 * 1.2)
    plt.tight_layout()
    plt.savefig(args.out)

if __name__ == '__main__':
    main()

