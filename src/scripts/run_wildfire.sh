#!/usr/bin/env bash
# RBE550 HW5 – Wildfire Orchestration Script
# Simple helper to run simulations, plot results, and make animations.

set -e  # Exit immediately if a command fails

# Default parameters
RUNS=5
DURATION=3600
SEED=1000
OUTDIR="results"

echo "------------------------------------------------------------"
echo " RBE550 HW5 Wildfire Simulation Runner"
echo "------------------------------------------------------------"
echo "Runs: $RUNS | Duration: $DURATION s | Seed: $SEED | Outdir: $OUTDIR"
echo ""

# Ensure output directory exists
mkdir -p "$OUTDIR"

# Step 1. Run all simulations
python3 -m sim.run_firefight --runs "$RUNS" --duration "$DURATION" --outdir "$OUTDIR" --seed_base "$SEED"

# Step 2. Plot compute-time comparison
if [ -f "$OUTDIR/compute_times.csv" ]; then
    python3 -m utils.metrics "$OUTDIR/compute_times.csv" --out "$OUTDIR/compute_times.png"
else
    echo "Warning: compute_times.csv not found; skipping plot generation."
fi

# Step 3. Optional animations (uncomment if ready)
# mkdir -p media
# python3 -m sim.animate --run "$OUTDIR/run_${SEED}.csv" --out media/wumpus_excerpt.mp4 --mode wumpus
# python3 -m sim.animate --run "$OUTDIR/run_${SEED}.csv" --out media/firetruck_excerpt.mp4 --mode firetruck

echo ""
echo "------------------------------------------------------------"
echo " All runs completed!"
echo " Summary:      $OUTDIR/summary.csv"
echo " Champion:     $OUTDIR/champion.txt"
echo " Compute Plot: $OUTDIR/compute_times.png"
echo "------------------------------------------------------------"

