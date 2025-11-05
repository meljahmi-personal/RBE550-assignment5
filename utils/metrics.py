# utils/metrics.py
#!/usr/bin/env python3
import argparse, csv
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('--out', default='results/compute_times.png')
    args = ap.parse_args()

    sums = [0.0, 0.0, 0.0]  # wumpus, prm_build, prm_queries
    with open(args.csv_path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            sums[0] += float(row['wumpus_planning_s'])
            sums[1] += float(row['firetruck_roadmap_s'])
            sums[2] += float(row['firetruck_queries_s'])

    labels = ['Wumpus planning', 'PRM build', 'PRM queries']
    plt.figure()
    plt.bar(labels, sums)
    plt.ylabel('CPU time (s)')
    plt.title('Compute time (sum over runs)')
    plt.tight_layout()
    plt.savefig(args.out)

if __name__ == '__main__':
    main()

