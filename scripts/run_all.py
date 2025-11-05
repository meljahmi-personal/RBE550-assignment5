# run_all.py
import subprocess

def run_sim(runs=5, duration=3600, outdir="results", seed=1000):
    subprocess.run([
        "python", "-m", "sim.run_firefight",
        "--runs", str(runs),
        "--duration", str(duration),
        "--outdir", outdir,
        "--seed_base", str(seed)
    ], check=True)

def make_plot(outdir="results"):
    subprocess.run([
        "python", "-m", "utils.metrics",
        f"{outdir}/compute_times.csv",
        "--out", f"{outdir}/compute_times.png"
    ], check=True)

def animate(run_csv, mode="firetruck"):
    subprocess.run([
        "python", "-m", "sim.animate",
        "--run", run_csv,
        "--out", f"media/{mode}.mp4",
        "--mode", mode
    ], check=True)

if __name__ == "__main__":
    run_sim()
    make_plot()

