#!/usr/bin/env python3
import os, sys, subprocess, glob
HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

from sim.run_firefight import main as run_firefight_main
from src_env.wildfire_world import WildfireWorld
from sim.animate import replay_run, replay_run_filtered
from utils.metrics import metrics_main
import argparse
import sys, os, subprocess
from sim.run_firefight import main as run_firefight_main
import glob, numpy as np
from PIL import Image
import imageio.v2 as imageio
from sim.animate import replay_run_tiles_only
from sim.animate import replay_full_world, replay_run_filtered, replay_run



def make_gif(glob_pattern, gif_path, sec_per_frame=0.7, stride=2):
    """
    Stream frames into a GIF, resizing to the first frame's size.
    'stride' reduces frame count & memory (2=use every other frame).
    """
    pngs = sorted(glob.glob(glob_pattern))
    if not pngs:
        print(f"[WARN] No frames match {glob_pattern}; skipping {gif_path}")
        return

    # stride to keep RAM/file-size in check
    pngs = pngs[::max(1, int(stride))]

    # target size from first frame
    first = imageio.imread(pngs[0])
    target_wh = (first.shape[1], first.shape[0])  # (w, h)

    os.makedirs(os.path.dirname(gif_path), exist_ok=True)
    with imageio.get_writer(gif_path, mode="I", duration=float(sec_per_frame)) as w:
        for p in pngs:
            arr = imageio.imread(p)
            h, w0 = arr.shape[:2]
            if (w0, h) != target_wh:
                arr = np.array(Image.fromarray(arr).resize(target_wh, Image.BILINEAR))
            w.append_data(arr)

    print(f"✅ GIF: {gif_path}  ({len(pngs)} frames @ {sec_per_frame:.2f}s)")


def generate_visuals(outdir: str, seeds: list[int]) -> None:
    """
    Produce animations for submission.
    - One unified GIF per run: wildfire_tiles_{seed}.gif (tiles change color, no trails)
    - Optional short excerpts for each player (wumpus/truck) to satisfy the rubric.
    """
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

        # --- world instance for consistent rendering ---
        w = WildfireWorld(seed=seed)

        # ---------- A) ONE UNIFIED GIF (like the class example) ----------
        tiles_dir = os.path.join(frames_root, f"tiles_{seed}")
        os.makedirs(tiles_dir, exist_ok=True)

        # Renders per-frame PNGs with tile-state colors + agent markers (no trails)
        replay_full_world(run_csv, w, outdir=tiles_dir)

        # Stitch into a single GIF
        make_gif(os.path.join(tiles_dir, "frame_*.png"),
                 os.path.join(gifs_root, f"wildfire_tiles_{seed}.gif"),
                 sec_per_frame=0.7)


        wumpus_dir = os.path.join(frames_root, f"frames_wumpus_{seed}")
        truck_dir  = os.path.join(frames_root, f"frames_truck_{seed}")
        replay_run_filtered(run_csv, w, outdir=wumpus_dir, keep_types={"ignite", "burned"})
        replay_run_filtered(run_csv, w, outdir=truck_dir,  keep_types={"extinguish"})

        make_gif(os.path.join(wumpus_dir, "frame_*.png"),
                 os.path.join(gifs_root, f"wumpus_excerpt_{seed}.gif"),
                 sec_per_frame=0.7)
        make_gif(os.path.join(truck_dir,  "frame_*.png"),
                 os.path.join(gifs_root, f"truck_excerpt_{seed}.gif"),
                 sec_per_frame=0.7)

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

