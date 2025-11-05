# geom/collision.py
import math

def _project(poly, ax, ay):
    """Project polygon onto axis (ax, ay). Returns (min, max) scalar interval."""
    v0 = ax*poly[0][0] + ay*poly[0][1]
    mn = mx = v0
    for (x,y) in poly[1:]:
        v = ax*x + ay*y
        if v < mn: mn = v
        if v > mx: mx = v
    return mn, mx

def poly_intersect_sat(polyA, polyB):
    """
    Separating Axis Theorem for convex polygons (works for rectangles/tetromino cells).
    Returns True if polygons overlap.
    """
    for poly in (polyA, polyB):
        for i in range(len(poly)):
            x1,y1 = poly[i]
            x2,y2 = poly[(i+1) % len(poly)]
            # outward normal of the edge is a separating axis candidate
            ax, ay = -(y2 - y1), (x2 - x1)
            mnA, mxA = _project(polyA, ax, ay)
            mnB, mxB = _project(polyB, ax, ay)
            if mxA < mnB or mxB < mnA:
                return False  # found a separating axis
    return True

def first_collision(vehicle_polys, obstacle_polys):
    """
    Given a list of vehicle polygons sampled along an edge (pose samples),
    return (k, obstacle_index) of the first collision, or (None, None) if safe.
    """
    for k, vpoly in enumerate(vehicle_polys):
        for j, op in enumerate(obstacle_polys):
            if poly_intersect_sat(vpoly, op):
                return k, j
    return None, None

