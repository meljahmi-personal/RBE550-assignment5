# RBE550 Assignment 5 — Wildfire Simulation


---

## Setup & Installation

### 1. Clone the Repository
```bash
git clone git@github.com:meljahmi-personal/RBE550-assignment5.git
cd RBE550-assignment5
```

### 2. Create and Activate a Virtual Environment
```bash
python3 -m venv hw5env
source hw5env/bin/activate    # On Linux/Mac
# or
hw5env\Scripts\activate     # On Windows
```

### 3. Install dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

---

## Running the Simulation

### Run a Single Simulation
To run one 600-second test simulation:
```bash
python3 scripts/run_all.py --runs 1 --duration 600
```

### Run All Five Seeds (Full Evaluation)
To generate five randomized wildfire runs for the final report:
```bash
python3 scripts/run_all.py --runs 5 --duration 3600
```

Each run produces:
- `results/run_<seed>.csv` — simulation timeline of events  
- `results/summary.csv` — per-run performance summary  
- `results/compute_times.csv` — CPU timing metrics  
- `results/gifs/` — simulation animations for report submission  

Example console output:
```
GIF: results/gifs/wildfire_tiles_1000.gif  (198 frames @ 0.70s)
GIF: results/gifs/wumpus_excerpt_1000.gif  (242 frames @ 0.70s)
GIF: results/gifs/truck_excerpt_1000.gif   (9 frames @ 0.70s)
```

---

## Output Files

| File / Directory | Description |
|------------------|-------------|
| `results/summary.csv` | Scoring table across all runs (Wumpus vs Firetruck) |
| `results/compute_times.csv` | CPU timing breakdown for both planners |
| `results/compute_times.png` | Visualization of computational resources |
| `results/gifs/` | Animated GIFs showing agent behavior and world evolution |
| `results/champion.txt` | Declares overall winner (truck / wumpus / none) |

---

## Understanding the GIF Outputs

### `wildfire_tiles_<seed>.gif`
Full simulation world view:
- Gray = intact tiles  
- Orange = burning tiles  
- Blue = extinguished tiles  
- Black = burned tiles  
Shows the entire fire evolution and extinguishing process (rubric 5.7).

### `wumpus_excerpt_<seed>.gif`
Shows the **Wumpus (A\*) planner** spreading fires across the field.  
Focuses on ignition and fire‑spread behavior (rubric 5.2).

### `truck_excerpt_<seed>.gif`
Shows the **Firetruck (PRM)** navigating the field to extinguish burning tiles.  
Demonstrates Ackermann‑constrained motion and roadmap‑based planning (rubric 5.3).

---

## Computational Performance

Performance metrics for each run are automatically logged in:
```
results/compute_times.csv
```

A bar chart of average computation time is also generated as:
```
results/compute_times.png
```

Example metrics:
| Component | Avg Time (s) | Description |
|------------|--------------|-------------|
| Wumpus (A*) | ~0.21 | Total planning time over all re‑plans |
| PRM Build | ~0.05 | Initial roadmap generation |
| PRM Queries | ~0.01 | Path planning queries per target |

---

##  Technical Overview

- **Wumpus (A\*) Planner:** Grid-based combinatorial search that selects adjacent intact obstacles to ignite.  
- **Firetruck (PRM) Planner:** Sampling‑based roadmap ensuring feasible Ackermann motion while reaching burning obstacles.  
- **World:** 250 m × 250 m map with tetromino obstacles (10% coverage).  
- **Fire Dynamics:** Fires spread within a 30 m radius after 10 s; extinguishing requires 5 s within 10 m of a burning obstacle.

---

##  Scoring System
| Event | Points | Agent |
|--------|---------|--------|
| Obstacle Ignited | +1 | Wumpus |
| Obstacle Burned | +1 | Wumpus |
| Obstacle Extinguished | +2 | Firetruck |

The overall **champion** (Wumpus or Firetruck) is determined from five runs.

---

##  Repository Structure
```
RBE550-assignment5/
├── agents    # Agent implementations (Wumpus, Firetruck)
├── geom      # Geometric primitives and utility shapes
├── plan      # A* and PRM planners      
├── post      # Post
├── README.md
├── report    # Report
├── scripts   # Automation scripts (run_all.py)
├── sim       # Simulation and visualization modules
├── src_env   # World generation and fire-state logic
├── utils     # Timing and analysis tools
└── vehicles  # Vehicle kinematics and physical models

```



**End of README**
