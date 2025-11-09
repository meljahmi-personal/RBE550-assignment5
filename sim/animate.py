# sim/animate.py
import math
import numpy as np
import matplotlib
from matplotlib.lines import Line2D
matplotlib.use("Agg")  # off-screen backend for image/GIF writing
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import imageio.v2 as imageio
import glob
from geom.polygons import oriented_box
import matplotlib.pyplot as plt
import csv
import os
import shutil
from PIL import Image
import re
try:
    from src_env.fire_state import (
        STATE_INTACT,
        STATE_BURNING,
        STATE_EXTINGUISHED,
        STATE_BURNED,
    )
except ImportError:
    # fallback defaults
    STATE_INTACT = "intact"
    STATE_BURNING = "burning"
    STATE_EXTINGUISHED = "extinguished"
    STATE_BURNED = "burned"



# module-level or created per replay
TRAIL_TRUCK = []
TRAIL_WUMPUS = []
CUR_GOAL_TRUCK = None  # (x, y)
CUR_GOAL_WUMPUS = None


STATE_COLOR = {
    "intact":        ("#666666", 0.35),
    "burning":       ("orange",   0.60),
    "extinguished":  ("#1f77b4",  0.40),  # blue
    "burned":        ("black",    0.90),  # BLACK like the example clip
}


def _parse_obs_idx(notes: str | None) -> int | None:
    if not notes:
        return None
    try:
        for tok in str(notes).replace(",", " ").split():
            if tok.startswith("obs="):
                return int(tok.split("=", 1)[1])
    except Exception:
        pass
    return None


def _parse_obs_index(notes: str) -> int | None:
    # notes like "obs=248"
    m = re.search(r"obs\s*=\s*(\d+)", notes or "")
    return int(m.group(1)) if m else None


def replay_run_tiles_only(csv_path, world, out_gif, sec_per_frame=0.7):
    """Rubric-style animation: tiles change color; no trails/paths are drawn."""
    # Keep a per-obstacle state cache we mutate as we step through events.
    state_by_idx = {i: STATE_INTACT for i in range(len(world.obstacles))}

    # Load events
    events = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            events.append(row)

    frames = []
    Wm = world.WORLD_SIZE_M

    # Pre-compute agent start markers (for legend)
    def cell_to_xy(rc):
        ci, cj = int(rc[1]), int(rc[0])
        x = (ci + 0.5) * world.CELL_SIZE_M
        y = (cj + 0.5) * world.CELL_SIZE_M
        return (x, y)

    wumpus_xy = cell_to_xy(world.wumpus_start) if hasattr(world, "wumpus_start") else None
    truck_xy  = (world.firetruck_start[0], world.firetruck_start[1]) if hasattr(world, "firetruck_start") else None

    for ev in events:
        t = float(ev["t"])
        et = ev["event"]
        ex = float(ev["x"]); ey = float(ev["y"])
        idx = _parse_obs_idx(ev.get("notes", ""))

        # mutate the obstacle state cache
        if idx is not None:
            if et == "ignite":       state_by_idx[idx] = STATE_BURNING
            elif et == "extinguish": state_by_idx[idx] = STATE_EXTINGUISHED
            elif et == "burned":     state_by_idx[idx] = STATE_BURNED

        # draw frame
        fig, ax = plt.subplots(figsize=(6,6))
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(0, Wm); ax.set_ylim(0, Wm)

        _draw_board_from_states(ax, world, state_by_idx)
        ax.set_title(f"t={t:.1f}s — {et}")

        # Always show legend entries (even if not on this frame)
        if wumpus_xy is not None:
            ax.plot(wumpus_xy[0], wumpus_xy[1], marker="x", markersize=6, color="orange", label="Wumpus start")
        if truck_xy is not None:
            ax.plot(truck_xy[0], truck_xy[1], marker="o", markersize=5, color="blue", label="Firetruck start")
        # “ghost” handles so legend always lists the event types
        ax.plot([], [], marker="*", color="orange", linestyle="None", label="ignite")
        ax.plot([], [], marker="*", color="blue",   linestyle="None", label="extinguish")
        ax.plot([], [], marker="*", color="black",  linestyle="None", label="burned")

        # Current event marker (skip init)
        if et != "init":
            color = {"ignite":"orange", "extinguish":"blue", "burned":"black"}.get(et, "red")
            ax.plot(ex, ey, marker="*", markersize=7, color=color)

        ax.legend(loc="upper right", fontsize=6)

        # Rasterize a consistent-sized frame
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)[..., :3].copy()
        plt.close(fig)
        frames.append(buf)

    if frames:
        imageio.mimsave(out_gif, frames, duration=float(sec_per_frame))
        print(f"✅ GIF: {out_gif}  ({len(frames)} frames @ {sec_per_frame:.2f}s)")
    else:
        print("⚠️ tiles-only: no frames created")



def replay_run_filtered(csv_path, world, outdir="frames_filtered", keep_types=None):
    os.makedirs(outdir, exist_ok=True)
    rows=[]
    with open(csv_path, newline="") as f:
        r=csv.DictReader(f)
        for row in r:
            if (keep_types is None) or (row["event"] in keep_types) or (row["event"]=="init"):
                rows.append(row)
    for i, ev in enumerate(rows):
        fig, ax = draw_world(world, title=f"t={float(ev['t']):.1f}s — {ev['event']}")
        if ev["event"] != "init":
            ax.plot(float(ev["x"]), float(ev["y"]), marker="*", color="orange" if ev["event"]=="ignite" else "red")
        fig.savefig(os.path.join(outdir, f"frame_{i:04d}.png"), dpi=200)
        plt.close(fig)


def _draw_board_from_states(ax, world, state_by_idx):
    """Draw obstacles using a per-obstacle state cache (index -> state)."""
    cell = world.CELL_SIZE_M
    Wm = world.WORLD_SIZE_M

    # grid
    n_cells = int(round(Wm / cell))
    xs = [i * cell for i in range(n_cells + 1)]
    ys = [j * cell for j in range(n_cells + 1)]
    for x in xs: ax.plot([x, x], [0, Wm], color="#dddddd", linewidth=0.5, zorder=0)
    for y in ys: ax.plot([0, Wm], [y, y], color="#dddddd", linewidth=0.5, zorder=0)

    # obstacles colored by current state
    for idx, obs in enumerate(world.obstacles):
        st = state_by_idx.get(idx, STATE_INTACT)
        if st == STATE_BURNING:
            face, alpha = "orange", 0.7
        elif st == STATE_BURNED:
            face, alpha = "black", 0.6
        elif st == STATE_EXTINGUISHED:
            face, alpha = "blue", 0.5
        else:
            face, alpha = "#666666", 0.35
        for (ci, cj) in obs.cells:
            x0 = ci * cell; y0 = cj * cell
            ax.add_patch(MplPoly([(x0,y0),(x0+cell,y0),(x0+cell,y0+cell),(x0,y0+cell)],
                                 closed=True, fill=True, facecolor=face,
                                 edgecolor="black", linewidth=0.7, alpha=alpha, zorder=1))
    # boundary
    ax.plot([0, Wm, Wm, 0, 0],[0, 0, Wm, Wm, 0], color="black", linewidth=1.5, zorder=2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_xticks(xs); ax.set_yticks(ys); ax.tick_params(labelsize=6)
    ax.text(5, Wm-5, "HW5 MAP", color="black", fontsize=8,
            bbox=dict(facecolor="white", edgecolor="black", alpha=0.8), zorder=10)


def draw_world_with_states(world, states, title=None):
    cell = world.CELL_SIZE_M
    Wm   = world.WORLD_SIZE_M
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, Wm); ax.set_ylim(0, Wm)

    # grid
    n = int(round(Wm / cell))
    xs = [i * cell for i in range(n + 1)]
    ys = [j * cell for j in range(n + 1)]
    for x in xs: ax.plot([x, x], [0, Wm], color="#dddddd", linewidth=0.5, zorder=0)
    for y in ys: ax.plot([0, Wm], [y, y], color="#dddddd", linewidth=0.5, zorder=0)

    # obstacles by state
    for idx, obs in enumerate(world.obstacles):
        face, alpha = STATE_COLOR.get(states[idx], STATE_COLOR["intact"])
        for (ci, cj) in obs.cells:
            x0, y0 = ci * cell, cj * cell
            ax.add_patch(MplPoly(
                [(x0,y0),(x0+cell,y0),(x0+cell,y0+cell),(x0,y0+cell)],
                closed=True, fill=True, facecolor=face, edgecolor="black",
                linewidth=0.6, alpha=alpha, zorder=1))

    # boundary + cosmetics
    ax.plot([0, Wm, Wm, 0, 0], [0, 0, Wm, Wm, 0], color="black", linewidth=1.3, zorder=2)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_xticks(xs); ax.set_yticks(ys); ax.tick_params(labelsize=6)
    ax.set_title(title or "Wildfire World")
    ax.text(5, Wm-5, "HW5 MAP",
            fontsize=8, color="black",
            bbox=dict(facecolor="white", edgecolor="black", alpha=0.8),
            zorder=10)
    return fig, ax


def replay_full_world(csv_path, world, outdir="frames_tiles"):
    """
    Render the whole run as frames that look like the class example:
      - tiles change color with state (intact/burning/extinguished/burned)
      - agents shown as simple markers (no trails, no path lines)
      - constant figure size so all PNGs have the same shape (GIF safe)

    Writes PNGs to `outdir/` (frame_0000.png, ...). Use make_gif(...) to stitch.
    """
    import csv, os
    os.makedirs(outdir, exist_ok=True)

    # defensive copy of states - will be mutated during replay
    for obs in world.obstacles:
        # normalize to strings 
        if getattr(obs, "state", None) is None:
            obs.state = "intact"

    # last known agent positions (meters)
    truck_xy  = None
    wumpus_xy = None

    # small helpers
    def cell_center_m(ij):
        ci, cj = ij
        s = world.CELL_SIZE_M
        return (cj * s + 0.5 * s, ci * s + 0.5 * s)

    # If the world carries starts, convert to meters for legend dots
    start_wumpus = None
    start_truck  = None
    if hasattr(world, "wumpus_start") and world.wumpus_start is not None:
        start_wumpus = cell_center_m(world.wumpus_start)
    if hasattr(world, "firetruck_start") and world.firetruck_start is not None:
        # firetruck_start could already be meters
        sv = world.firetruck_start
        start_truck = (float(sv[0]), float(sv[1])) if len(sv) >= 2 else None

    # load events
    rows = []
    with open(csv_path, newline="") as f:
        R = csv.DictReader(f)
        rows = list(R)

    # replay and draw
    for k, ev in enumerate(rows):
        if k % 5 != 0:    # keep one frame every 5 events to avoid memory exhaustion
            continue

        t = float(ev["t"]); et = ev["event"]

        # keep agent positions up to date (no trails, just current)
        if et == "pos_truck":
            truck_xy = (float(ev["x"]), float(ev["y"]))
        elif et == "pos_wumpus":
            wumpus_xy = (float(ev["x"]), float(ev["y"]))

        # update tile states from events
        elif et in ("ignite", "burned", "extinguish"):
            note = ev.get("notes", "")
            # expect notes like "obs=123"
            idx = None
            if note and "obs=" in note:
                try:
                    idx = int(note.split("obs=")[1].split()[0])
                except Exception:
                    idx = None
            if idx is not None and 0 <= idx < len(world.obstacles):
                if et == "ignite":
                    world.obstacles[idx].state = "burning"
                elif et == "burned":
                    world.obstacles[idx].state = "burned"
                elif et == "extinguish":
                    world.obstacles[idx].state = "extinguished"

        # draw a frame for every row (nice cadence and deterministic)
        fig, ax = draw_world(world, title=f"t={t:.1f}s — {et}")

        # overlay CURRENT agent markers only (no polylines)
        if start_wumpus is not None:
            ax.plot(start_wumpus[0], start_wumpus[1], marker="x",
                    markersize=6, color="orange", label="Wumpus start")
        if start_truck is not None:
            ax.plot(start_truck[0], start_truck[1], marker="o",
                    markersize=5, color="blue", label="Firetruck start")

        if wumpus_xy is not None:
            ax.plot(wumpus_xy[0], wumpus_xy[1], marker="^", markersize=6,
                    color="orange")
        if truck_xy is not None:
            ax.plot(truck_xy[0], truck_xy[1], marker=">", markersize=6,
                    color="steelblue")
                    
                    
        state_handles = [
            Line2D([0], [0], marker='s', linestyle='None', color='w',
                   markerfacecolor='#ff8c00', label='burning', markersize=6),
            Line2D([0], [0], marker='s', linestyle='None', color='w',
                   markerfacecolor='#1f77b4', label='extinguished', markersize=6),
            Line2D([0], [0], marker='s', linestyle='None', color='w',
                   markerfacecolor='#000000', label='burned', markersize=6),
        ]

        start_handles = []
        start_handles.append(Line2D([0],[0], marker='x', linestyle='None', color='orange', label='Wumpus start'))
        start_handles.append(Line2D([0],[0], marker='o', linestyle='None', color='blue',   label='Firetruck start'))

        try:
            ax.legend(handles=start_handles + state_handles, loc="upper right", fontsize=6)
        except Exception:
            pass


        # IMPORTANT: keep image size identical across frames (no bbox_inches="tight")
        fig.savefig(os.path.join(outdir, f"frame_{k:04d}.png"), dpi=200)
        plt.close(fig)
        
# ---------------------------------------------------------------------------



def draw_world(world, title=None):
    """
    Pretty world snapshot for report:
    - light gray grid (each cell is CELL_SIZE_M meters)
    - filled obstacle cells colored by fire state
    - world boundary
    - axis ticks in meters
    """

    cell = world.CELL_SIZE_M          # e.g. 5.0 m
    Wm   = world.WORLD_SIZE_M         # e.g. 250.0 m

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, Wm)
    ax.set_ylim(0, Wm)

    # --- grid lines ---
    n_cells = int(round(Wm / cell))
    xs = [i * cell for i in range(n_cells + 1)]
    ys = [j * cell for j in range(n_cells + 1)]

    for x in xs:
        ax.plot([x, x], [0, Wm], color="#dddddd", linewidth=0.5, zorder=0)
    for y in ys:
        ax.plot([0, Wm], [y, y], color="#dddddd", linewidth=0.5, zorder=0)

    
    # --- obstacles ---
    for obs in world.obstacles:
        st = getattr(obs, "state", "intact")
        if st == "burning":
            face, alpha = "orange", 0.6
        elif st == "burned":
            face, alpha = "black", 0.6         # <- black (not red)
        elif st == "extinguished":
            face, alpha = "#1f77b4", 0.5       # <- blue (same as legend)
        else:
            face, alpha = "#666666", 0.35

        for (ci, cj) in obs.cells:
            x0 = ci * cell; y0 = cj * cell
            ax.add_patch(
                MplPoly([(x0,y0),(x0+cell,y0),(x0+cell,y0+cell),(x0,y0+cell)],
                        closed=True, fill=True,
                        facecolor=face, edgecolor="black",
                        linewidth=0.6, alpha=alpha, zorder=1)
            )


    # --- boundary box ---
    ax.plot([0, Wm, Wm, 0, 0],
            [0, 0, Wm, Wm, 0],
            color="black",
            linewidth=1.5,
            zorder=2)

    # --- cosmetics / text ---
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if title is None:
        title = "Wildfire World (seeded map)"
    ax.set_title(title)

    # ticks at every grid cell so grader sees scale
    ax.set_xticks(xs)
    ax.set_yticks(ys)
    ax.tick_params(labelsize=6)

    # watermark tag to prove final renderer
    ax.text(
        5, Wm - 5,
        "HW5 MAP",
        color="black",
        fontsize=8,
        bbox=dict(facecolor="white", edgecolor="black", alpha=0.8),
        zorder=10
    )

    return fig, ax


def _arrow(ax, p, g, width=0.6, head=5.0, frac=0.33):
    """Short arrow from point p toward goal g."""
    if not p or not g:
        return
    dx, dy = (g[0] - p[0]), (g[1] - p[1])
    ax.arrow(p[0], p[1], frac * dx, frac * dy,
             width=width, head_width=head, length_includes_head=True, zorder=8)


def _iter_resize(png_paths, target_wh=None):
    """Yield frames resized to target_wh (set by first frame)."""
    for p in png_paths:
        img = imageio.imread(p)
        h, w = img.shape[:2]
        if target_wh is None:
            target_wh = (w, h)
        elif (w, h) != target_wh:
            img = np.array(Image.fromarray(img).resize(target_wh, Image.BILINEAR))
        yield img, target_wh


def replay_run(csv_path, world, outdir="frames", sec_per_frame=0.7):
    """
    Render one run CSV as PNG frames + GIF with:
      - start markers
      - trails for truck (solid) and wumpus (dashed)
      - goal arrows for both agents
      - optional sparse A* path breadcrumbs (if 'path_wumpus' rows exist)
      - event star for ignite/extinguish/burned
    """
    shutil.rmtree(outdir, ignore_errors=True)
    os.makedirs(outdir, exist_ok=True)

    # Load all rows
    events = []
    with open(csv_path, newline="") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            # be tolerant of blanks
            try:
                row["t"] = float(row["t"])
                row["x"] = float(row["x"])
                row["y"] = float(row["y"])
            except Exception:
                continue
            events.append(row)

    # Helpers to convert stored starts to meters if needed
    def _cell_to_xy(ci, cj):
        return ((ci + 0.5) * world.CELL_SIZE_M, (cj + 0.5) * world.CELL_SIZE_M)

    def _robust_start_xy(val):
        if not hasattr(world, "WORLD_SIZE_M"):
            return None
        try:
            vals = list(val)
            a, b = float(vals[0]), float(vals[1])
        except Exception:
            return None
        n_cells = int(round(world.WORLD_SIZE_M / world.CELL_SIZE_M))
        # assume grid if both < n_cells
        return _cell_to_xy(a, b) if (abs(a) <= n_cells and abs(b) <= n_cells) else (a, b)

    wumpus_xy = _robust_start_xy(getattr(world, "wumpus_start", None))
    truck_xy  = _robust_start_xy(getattr(world, "firetruck_start", None))

    # Per-run accumulators (do NOT keep globals; reset per replay)
    trail_truck = []
    trail_wumpus = []
    cur_goal_truck = None
    cur_goal_wumpus = None
    path_wumpus_pts = []   # optional sparse breadcrumbs

    for i, ev in enumerate(events):
        etype = ev["event"]
        ex, ey = ev["x"], ev["y"]

        # --- Update state BEFORE drawing (so trails/goals include this row) ---
        if etype == "pos_truck":
            trail_truck.append((ex, ey))
        elif etype == "pos_wumpus":
            trail_wumpus.append((ex, ey))
        elif etype == "goal_truck":
            cur_goal_truck = (ex, ey)
        elif etype == "goal_wumpus":
            cur_goal_wumpus = (ex, ey)
        elif etype == "path_wumpus":
            path_wumpus_pts.append((ex, ey))
        # leave ignite/extinguish/burned for the star marker

        # --- Base map ---
        fig, ax = draw_world(world, title=f"t={ev['t']:.1f}s — {etype}")

        # starts (once, for context)
        if wumpus_xy is not None:
            ax.plot(wumpus_xy[0], wumpus_xy[1], marker="x", markersize=7,
                    color="orange", label="Wumpus start", zorder=7)
        if truck_xy is not None:
            ax.plot(truck_xy[0], truck_xy[1], marker="o", markersize=6,
                    color="blue", label="Firetruck start", zorder=7)

        # --- Trails (so far) ---
        if len(trail_truck) >= 2:
            xs, ys = zip(*trail_truck)
            ax.plot(xs, ys, linewidth=2.0, zorder=6)  # solid trail
            # current truck dot
            ax.scatter([xs[-1]], [ys[-1]], s=28, zorder=9)
        if len(trail_wumpus) >= 2:
            xs, ys = zip(*trail_wumpus)
            ax.plot(xs, ys, linewidth=1.5, linestyle="--", zorder=6)  # dashed trail
            # current wumpus "x"
            ax.scatter([xs[-1]], [ys[-1]], s=28, marker="x", zorder=9)

        # --- Goal arrows ---
        if trail_truck and cur_goal_truck:
            _arrow(ax, trail_truck[-1], cur_goal_truck)
        if trail_wumpus and cur_goal_wumpus:
            _arrow(ax, trail_wumpus[-1], cur_goal_wumpus)

        # --- Optional A* breadcrumbs (sparse) ---
        if path_wumpus_pts:
            xs, ys = zip(*path_wumpus_pts)
            ax.scatter(xs, ys, s=10, zorder=8)

        # --- Current event star for clarity (skip init/pos/goal/path) ---
        if etype in ("ignite", "extinguish", "burned"):
            star_color = ("orange" if etype == "ignite"
                          else "blue" if etype == "extinguish"
                          else "red")
            ax.plot(ex, ey, marker="*", markersize=7, color=star_color, zorder=9)

        ax.legend(loc="upper right", fontsize=6)
        fig.savefig(os.path.join(outdir, f"frame_{i:04d}.png"), dpi=200, bbox_inches="tight")
        plt.close(fig)


    # Build GIF with constant memory: stream frames and optionally stride
    pngs = sorted(glob.glob(os.path.join(outdir, "frame_*.png")))
    if pngs:
        frame_stride = 2
        pngs = pngs[::frame_stride]

        # determine target size from first frame
        first = imageio.imread(pngs[0])
        target_wh = (first.shape[1], first.shape[0])  # (w, h)

        gif_path = os.path.join("results", f"{os.path.basename(outdir)}.gif")
        with imageio.get_writer(gif_path, mode="I", duration=float(sec_per_frame)) as w:
            for p in pngs:
                arr = imageio.imread(p)
                h, w0 = arr.shape[:2]
                if (w0, h) != target_wh:
                    arr = np.array(Image.fromarray(arr).resize(target_wh, Image.BILINEAR))
                w.append_data(arr)

        print(f"🎬 GIF: {gif_path}  ({len(pngs)} frames @ {sec_per_frame:.2f}s)")
    else:
        print("⚠️ No frames found, skipping GIF creation.")





    print(f"✅ Saved {len(events)} frames to {outdir}/")



def draw_scene(ax, world, obstacles, bays, veh_poly=None):
    """Draw obstacles, parking bays, and (optionally) one vehicle polygon."""
    ax.clear()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(0, world.grid_size_cells * world.cell_size_m)
    ax.set_ylim(0, world.grid_size_cells * world.cell_size_m)
    
    # Draw outer world boundary (visual reference)
    L = world.grid_size_cells * world.cell_size_m
    ax.plot([0, L, L, 0, 0],
            [0, 0, L, L, 0],
            color="black", linewidth=1.5)

    # 🔹 Add gridlines and ticks for visual clarity (each cell = 3 m)
    ax.set_xticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
    ax.set_yticks([i * world.cell_size_m for i in range(world.grid_size_cells + 1)])
    ax.grid(True, linewidth=0.5, color="#dddddd")

    # obstacles (filled)
    for poly in obstacles:
        ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))

    # parking bays (outlines)
    for bay in bays:
        ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

    # vehicle polygon (outline)
    if veh_poly is not None:
        ax.add_patch(MplPoly(veh_poly, closed=True, fill=False, linewidth=2))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Valet — static scene")
    


def save_png(world, obstacles, bays, vehicle_polygon, out_path):
    """
    Save a static scene PNG with one vehicle polygon (start pose).
    Minimal, explicit, no helpers.
    """
    W = world.grid_size_cells * world.cell_size_m
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, W); ax.set_ylim(0, W)

    # Obstacles (filled)
    for poly in obstacles:
        ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))

    # Parking bay(s) (outline)
    for bay in bays:
        ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

    # Vehicle polygon (outline)
    if vehicle_polygon is not None:
        ax.add_patch(MplPoly(vehicle_polygon, closed=True, fill=False, linewidth=2, edgecolor="red"))
        # small heading marker (optional)
        cx = sum(p[0] for p in vehicle_polygon) / 4.0
        cy = sum(p[1] for p in vehicle_polygon) / 4.0


    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Valet — static scene")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def save_path_png(world, obstacles, bays, path_xy, out_path, vehicle_polygons=None):
    """
    Save a scene PNG with an (x,y) polyline and optional one-or-more vehicle polygons.

    Args:
        world: world object (for bounds)
        obstacles: list of obstacle polygons
        bays: list of parking-bay polygons
        path_xy: list of (x,y) points to draw as a polyline
        out_path: PNG filename
        vehicle_polygons: optional list of polygons to overlay
                          (e.g., [truck_poly, trailer_poly])
    """
    W = world.grid_size_cells * world.cell_size_m
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, W); ax.set_ylim(0, W)

    for poly in obstacles:
        ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))
    for bay in bays:
        ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

    if path_xy:
        xs, ys = zip(*path_xy)
        ax.plot(xs, ys, linewidth=2)

    if vehicle_polygons:
        for poly in vehicle_polygons:
            ax.add_patch(MplPoly(poly, closed=True, fill=False, linewidth=2, edgecolor="red"))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Valet — planned path")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# NOTE on the "*" below:
# In Python, placing "*" in a function's parameter list means that
# all arguments appearing *after* it must be passed by keyword name.
# This prevents mistakes where a value (like a float for vehicle_length_meters)
# is accidentally interpreted as a positional argument (e.g., stride).
# In short — it makes argument passing safer and clearer, without changing behavior.

def save_gif_frames(
    world,
    obstacles,
    bays,
    path,
    out="valet.gif",
    stride=1,
    *,
    vehicle_length_meters=None,
    vehicle_width_meters=None,
    frame_delay=0.10,
    trailer_path=None,
    trailer_length_meters=None,
    trailer_width_meters=None,
):
    """
    Save an animated GIF of the vehicle moving along `path`.

    Args:
        world: World object (for bounds).
        obstacles: list of obstacle polygons.
        bays: list of parking bay polygons.
        path: list of (x, y, theta) vehicle poses.
        out (str): output GIF filename.
        stride (int | float): sample every k-th pose (float is cast to int).
        vehicle_length_meters (float|None): if provided, draws a rectangle footprint.
        vehicle_width_meters  (float|None): if provided, draws a rectangle footprint.
        frame_delay (float): frame duration in seconds.
        trailer_path (list|None): list of trailer poses aligned with vehicle poses.
        trailer_length_meters (float|None): trailer length for rectangle drawing.
        trailer_width_meters  (float|None): trailer width  for rectangle drawing.
    """


    # --- make stride robust (avoid float-in-range crash) ---
    try:
        stride_int = int(round(stride))
    except Exception:
        stride_int = 1
    if stride_int < 1:
        stride_int = 1

    imgs = []
    Wm = world.grid_size_cells * world.cell_size_m  # map is square

    # helper to render one frame
    def render_frame(k_idx):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(0, Wm)
        ax.set_ylim(0, Wm)

        # draw obstacles
        for poly in obstacles:
            ax.add_patch(MplPoly(poly, closed=True, fill=True, alpha=0.25))

        # draw bays
        for bay in bays:
            ax.add_patch(MplPoly(bay, closed=True, fill=False, linewidth=2))

        # path-so-far
        xs = [p[0] for p in path[:k_idx+1]]
        ys = [p[1] for p in path[:k_idx+1]]
        ax.plot(xs, ys, linewidth=2)

        # vehicle at current pose
        x, y, th = path[k_idx]
        if (vehicle_length_meters is not None) and (vehicle_width_meters is not None):
            vpoly = oriented_box((x, y), vehicle_length_meters, vehicle_width_meters, th)
            ax.add_patch(MplPoly(vpoly, closed=True, fill=False, linewidth=2))
            # heading tick (red)
            hx, hy = x + 0.8 * math.cos(th), y + 0.8 * math.sin(th)
            ax.plot([x, hx], [y, hy], linewidth=2, color='red')
        else:
            # dot fallback
            ax.plot(x, y, marker='o', markersize=6)
            hx, hy = x + 0.8 * math.cos(th), y + 0.8 * math.sin(th)
            ax.plot([x, hx], [y, hy], linewidth=2)

        # optional trailer
        if trailer_path and (trailer_length_meters is not None) and (trailer_width_meters is not None):
            if k_idx < len(trailer_path):
                xt, yt, tht = trailer_path[k_idx]
                tpoly = oriented_box((xt, yt), trailer_length_meters, trailer_width_meters, tht)
                ax.add_patch(MplPoly(tpoly, closed=True, fill=False, linewidth=2, linestyle='--'))

        # rasterize
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        rgba = buf.reshape(h, w, 4)
        rgb = rgba[..., :3].copy()
        plt.close(fig)
        return rgb

    # iterate frames
    for k in range(0, len(path), stride_int):
        imgs.append(render_frame(k))

    # write GIF (frame_delay seconds per frame)
    imageio.mimsave(out, imgs, duration=float(frame_delay))

