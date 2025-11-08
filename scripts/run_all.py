#!/usr/bin/env python3
import os, sys, subprocess, glob
HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

from sim.run_firefight import main as run_firefight_main
from src_env.wildfire_world import WildfireWorld
from sim.animate import replay_run, replay_run_filtered
import imageio.v2 as imageio
from utils.metrics import metrics_main
import argparse
import sys, os, subprocess
from sim.run_firefight import main as run_firefight_main


def make_gif(frames_pattern: str, gif_path: str, sec_per_frame: float = 0.7) -> None:
    """Read PNG frames matching pattern and write a single GIF to gif_path."""
    frames = sorted(glob.glob(frames_pattern))
    if not frames:
        print(f"[WARN] No frames matched {frames_pattern}")
        return

    if len(frames) < 2:
        print(f"[WARN] Skipping GIF {gif_path}: only {len(frames)} frames.")
        return

    os.makedirs(os.path.dirname(gif_path), exist_ok=True)  # only for GIF folder
    imgs = [imageio.imread(p) for p in frames]
    imageio.mimsave(gif_path, imgs, duration=float(sec_per_frame))
    print(f"✅ GIF: {gif_path}  ({len(frames)} frames @ {sec_per_frame:.2f}s)")


def generate_visuals(outdir: str, seeds: list[int]) -> None:
    """Write PNG frames to results/frames/* and GIFs to results/gifs/*."""
    frames_root = os.path.join(outdir, "frames")
    gifs_root   = os.path.join(outdir, "gifs")
    os.makedirs(frames_root, exist_ok=True)
    os.makedirs(gifs_root,   exist_ok=True)

    from src_env.wildfire_world import WildfireWorld

    for seed in seeds:
        run_csv = os.path.join(outdir, f"run_{seed}.csv")
        if not os.path.exists(run_csv):
            print(f"[WARN] Missing {run_csv}, skipping seed {seed}.")
            continue

    
        w = WildfireWorld(seed=seed)

        # 1) Full run frames (PNGs only)
        frames_dir = os.path.join(frames_root, f"frames_{seed}")
        os.makedirs(frames_dir, exist_ok=True)
        replay_run(run_csv, w, outdir=frames_dir)  # <-- creates only PNGs

        # 2) Filtered frames (PNGs only)
        wumpus_dir = os.path.join(frames_root, f"frames_wumpus_{seed}")
        truck_dir  = os.path.join(frames_root, f"frames_truck_{seed}")
        replay_run_filtered(run_csv, w, outdir=wumpus_dir, keep_types={"ignite","burned"})
        replay_run_filtered(run_csv, w, outdir=truck_dir,  keep_types={"extinguish"})

        # 3) Make GIFs (consume PNGs -> write GIFs)
        make_gif(os.path.join(frames_dir,  "frame_*.png"), os.path.join(gifs_root, f"wildfire_run{seed}.gif"),  sec_per_frame=0.7)
        make_gif(os.path.join(wumpus_dir, "frame_*.png"), os.path.join(gifs_root, f"wumpus_excerpt_{seed}.gif"), sec_per_frame=0.7)
        make_gif(os.path.join(truck_dir,  "frame_*.png"), os.path.join(gifs_root, f"truck_excerpt_{seed}.gif"),  sec_per_frame=0.7)

    print("🎬 PNGs in results/frames/, GIFs in results/gifs/")


def run_all():

    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--duration', type=int, default=3600)
    ap.add_argument('--outdir', type=str, default='results')
    ap.add_argument('--seed_base', type=int, default=1000)
    ap.add_argument('--wall_time', type=float, default=0.0)
    args = ap.parse_args()

    # Ensure output directory exists
    os.makedirs(args.outdir, exist_ok=True)

    # 1) Run the simulation for all seeds (single call)
    cmd = [
        sys.executable, "-m", "sim.run_firefight",
        "--runs", str(args.runs),
        "--duration", str(args.duration),
        "--outdir", args.outdir,
        "--seed_base", str(args.seed_base),
        "--wall_time", str(args.wall_time),
    ]
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)

    # 2) Plot compute times WITHOUT subprocess
    csv_path = os.path.join(args.outdir, "compute_times.csv")
    png_path = os.path.join(args.outdir, "compute_times.png")
    if os.path.exists(csv_path):
        argv_bak = sys.argv
        try:
            sys.argv = ["metrics", csv_path, "--out", png_path]
            metrics_main()  # assumes sim/metrics.py exposes metrics_main
        finally:
            sys.argv = argv_bak
    else:
        print(f"[WARN] {csv_path} not found; skipping compute time plot.")

    # 3) Champion table (5.4): tally scores from per-run CSVs
    import csv as _csv
    champion_path = os.path.join(args.outdir, "champion.txt")
    seeds = [args.seed_base + i for i in range(args.runs)]
    with open(champion_path, "w") as out:
        for s in seeds:
            run_csv = os.path.join(args.outdir, f"run_{s}.csv")
            if not os.path.exists(run_csv):
                print(f"[WARN] Missing {run_csv}, skipping seed {s}.")
                continue
            with open(run_csv, newline="") as f:
                rows = list(_csv.reader(f))
            ign = sum(1 for r in rows if len(r) > 1 and r[1] == "ignite")
            brn = sum(1 for r in rows if len(r) > 1 and r[1] == "burned")
            ext = sum(2 for r in rows if len(r) > 1 and r[1] == "extinguish")
            out.write(f"run_{s}.csv  Wumpus={ign}+{brn}={ign+brn}  Truck={ext}\n")

    # 4) Render frames + make GIFs once for the seeds we actually ran
    generate_visuals(args.outdir, seeds)

    print("🎬 PNGs in results/frames/, GIFs in results/gifs/")
    print("📊 Scores in results/champion.txt")
    print("Done. Check results/ and results/{frames,gifs}/.")


if __name__ == "__main__":
    run_all()

