#!/usr/bin/env python3
import os, sys, subprocess, glob
HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)

from sim.run_firefight import main as run_firefight_main
from src_env.wildfire_world import WildfireWorld
from sim.animate import replay_run, replay_run_filtered
import imageio.v2 as imageio
from utils.metrics import main as metrics_main


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
    seed_base = 1000
    runs = 5
    outdir = "results"

    # 1) Run the simulation for all seeds
    argv_bak = sys.argv
    try:
        sys.argv = [
            "sim.run_firefight",
            "--runs", str(runs),
            "--duration", "3600",
            "--outdir", outdir,
            "--seed_base", str(seed_base),
        ]
        run_firefight_main()  # writes run_1000..run_1004.csv, summary.csv, champion.txt, compute_times.csv
    finally:
        sys.argv = argv_bak

    # 2) Plot compute times WITHOUT subprocess
    csv_path = os.path.join(outdir, "compute_times.csv")
    png_path = os.path.join(outdir, "compute_times.png")
    if os.path.exists(csv_path):
        argv_bak = sys.argv
        try:
            sys.argv = ["metrics", csv_path, "--out", png_path]
            metrics_main()
        finally:
            sys.argv = argv_bak
    else:
        print(f"Warning: {csv_path} not found; skipping plot.")

    # 3) Render frames + make GIFs once
    seeds = [seed_base + i for i in range(runs)]
    generate_visuals(outdir, seeds)

    print("Done. Check results/ and results/{frames,gifs}/.")

    
if __name__ == "__main__":
    run_all()

