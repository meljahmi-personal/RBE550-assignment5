#!/usr/bin/env python3
import csv, sys
import matplotlib.pyplot as plt
from collections import defaultdict

def _pick_keys(fieldnames):
    """
    Return a tuple of keys (wumpus, prm_build, prm_query) that exist in the CSV.
    Supports both new and legacy column names.
    """
    # New schema (as suggested in sim/run_firefight.py patch)
    new = ("t_wumpus_total", "t_prm_build", "t_prm_query_total")
    if all(k in fieldnames for k in new):
        return new

    # Legacy schema used in some earlier versions
    legacy = ("wumpus_planning_s", "firetruck_roadmap_s", "firetruck_queries_s")
    if all(k in fieldnames for k in legacy):
        return legacy

    # Fallback: try to guess by partial matches
    def find(prefixes):
        for fn in fieldnames:
            for p in prefixes:
                if p in fn:
                    yield fn
                    break

    cand_w = list(find(["wumpus", "astar", "w_total", "tw"]))
    cand_b = list(find(["build", "roadmap", "prm_b"]))
    cand_q = list(find(["query", "queries", "prm_q"]))
    if cand_w and cand_b and cand_q:
        return (cand_w[0], cand_b[0], cand_q[0])

    raise KeyError("Could not identify timing columns. Fields present: " + ", ".join(fieldnames))

def metrics_main():
    if len(sys.argv) < 2:
        print("usage: metrics <compute_times.csv> --out <png>"); return
    csv_path = sys.argv[1]
    out = "compute_times.png"
    if "--out" in sys.argv:
        out = sys.argv[sys.argv.index("--out")+1]

    sums = defaultdict(float)
    with open(csv_path, newline="") as f:
        r = csv.DictReader(f)
        w_key, b_key, q_key = _pick_keys(r.fieldnames)

        for row in r:
            def safe_float(k):
                try:
                    return float(row.get(k, 0.0) or 0.0)
                except ValueError:
                    return 0.0
            sums["Wumpus A*"] += safe_float(w_key)
            sums["PRM build"] += safe_float(b_key)
            sums["PRM query"] += safe_float(q_key)

    labels = list(sums.keys())
    vals = [sums[k] for k in labels]
    # Avoid empty/zero plot issues
    if not any(vals):
        vals = [1e-6, 1e-6, 1e-6]

    plt.figure()
    plt.bar(labels, vals)
    plt.ylabel("CPU time (s)")
    plt.title("Total compute time across runs")
    plt.tight_layout()
    plt.savefig(out)
    print(f"[OK] wrote {out}")

if __name__ == '__main__':
    metrics_main()

