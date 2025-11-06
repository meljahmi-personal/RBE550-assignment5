# sim/animate.py
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")  # off-screen backend for image/GIF writing
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import imageio.v2 as imageio
import glob
from geom.polygons import oriented_box
import matplotlib.pyplot as plt
import csv
import os




def replay_run_filtered(csv_path, world, outdir="frames_filtered", keep_types=None):
    import csv, os
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


def draw_world(world, title=None):
    """
    Pretty world snapshot for report:
    - light gray grid (each cell is CELL_SIZE_M meters)
    - filled obstacle cells colored by fire state
    - world boundary
    - axis ticks in meters
    - 'HW5 MAP' tag in corner so we know it's rendered with the final code
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
            face = "orange"; alpha = 0.6
        elif st == "burned":
            face = "red"; alpha = 0.3
        elif st == "extinguished":
            face = "blue"; alpha = 0.4
        else:
            face = "#666666"; alpha = 0.35

        for (ci, cj) in obs.cells:
            x0 = ci * cell
            y0 = cj * cell
            square_xy = [
                (x0,        y0       ),
                (x0+cell,   y0       ),
                (x0+cell,   y0+cell  ),
                (x0,        y0+cell  ),
            ]
            ax.add_patch(
                MplPoly(
                    square_xy,
                    closed=True,
                    fill=True,
                    facecolor=face,
                    edgecolor="black",
                    linewidth=0.7,
                    alpha=alpha,
                    zorder=1,
                )
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


def replay_run(csv_path, world, outdir="frames"):
    """
    Read a run_XXXX.csv, render the world, and overlay:
    - Wumpus start (orange X)
    - Firetruck start (blue dot)
    - Logged event (red star if not 'init')
    Save one PNG per row into outdir/.
    This version is defensive about coordinate formats.
    """
    os.makedirs(outdir, exist_ok=True)

    # load log rows
    events = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            events.append(row)

    # helper 1: convert a grid cell (ci,cj) -> meters
    def cell_to_xy_from_cell(cell_ij):
        # cell_ij might be (ci, cj) or (ci, cj, theta)
        if len(cell_ij) >= 2:
            ci = cell_ij[0]
            cj = cell_ij[1]
        else:
            raise ValueError("cell_ij is too short")

        x = (ci + 0.5) * world.CELL_SIZE_M
        y = (cj + 0.5) * world.CELL_SIZE_M
        return (x, y)

    # helper 2: convert a pose that might ALREADY be meters
    # We detect the unit by magnitude:
    # - grid index is usually < 100
    # - meters could be up to WORLD_SIZE_M (~250)
    def robust_start_to_xy(start_val):
        # start_val could be tuple/list like (i,j) or (i,j,theta)
        # or maybe already (x,y) or (x,y,theta)
        vals = list(start_val)

        if len(vals) < 2:
            # can't parse
            return None

        a = float(vals[0])
        b = float(vals[1])

        # heuristic: if a or b are larger than the world size in meters,
        # that makes no sense, but if both are < ~n_cells assume grid.
        # Number of cells on a side:
        n_cells_est = int(round(world.WORLD_SIZE_M / world.CELL_SIZE_M))

        if abs(a) <= n_cells_est and abs(b) <= n_cells_est:
            # looks like grid indices
            return cell_to_xy_from_cell(vals)
        else:
            # looks like meters already (x,y)
            return (a, b)

    # precompute agent starts
    wumpus_xy = robust_start_to_xy(world.wumpus_start) \
                if hasattr(world, "wumpus_start") else None
    truck_xy  = robust_start_to_xy(world.firetruck_start) \
                if hasattr(world, "firetruck_start") else None

    for idx, ev in enumerate(events):
        t = float(ev["t"])
        etype = ev["event"]
        ex = float(ev["x"])
        ey = float(ev["y"])

        # draw base map
        fig, ax = draw_world(world, title=f"t={t:.1f}s — {etype}")

        # Wumpus start marker
        if wumpus_xy is not None:
            ax.plot(
                wumpus_xy[0], wumpus_xy[1],
                marker="x", markersize=6, color="orange",
                label="Wumpus start"
            )

        # Firetruck start marker
        if truck_xy is not None:
            ax.plot(
                truck_xy[0], truck_xy[1],
                marker="o", markersize=5, color="blue",
                label="Firetruck start"
            )

        # current event (skip 'init' so we don't spam)
        if etype != "init":
            ax.plot(
                ex, ey,
                marker="*", markersize=6, color="red",
                label=f"{etype} event"
            )

        # always try to show a legend 
        ax.legend(loc="upper right", fontsize=6)

        frame_path = os.path.join(outdir, f"frame_{idx:04d}.png")
        fig.savefig(frame_path, dpi=200, bbox_inches="tight")
        plt.close(fig)
        
        
    frames_dir = "frames_1000"
    gif_path = os.path.join("results", "wildfire_run1000.gif")

    # collect and sort frame file names
    pngs = sorted(glob.glob(os.path.join(frames_dir, "frame_*.png")))

    # build gif (duration = seconds per frame)
    if pngs:
        imgs = [imageio.imread(p) for p in pngs]
        imageio.mimsave(gif_path, imgs, duration=0.4)
        print(f"✅ Created animated GIF: {gif_path}")
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

