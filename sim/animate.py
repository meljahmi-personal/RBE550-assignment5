# sim/animate.py
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")  # off-screen backend for image/GIF writing
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import imageio.v2 as imageio
from geom.polygons import oriented_box


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
        # we can infer a short heading if caller wants, but keeping minimal here

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
    import math
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly
    import imageio.v2 as imageio
    from geom.polygons import oriented_box  # local import to avoid cycles

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

