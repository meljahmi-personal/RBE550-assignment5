# geom/polygons.py
import math

def oriented_box(center, length, width, theta):
    """
    Return a 4-vertex polygon for a rectangle centered at 'center' with heading 'theta'.
    Long side = length (front/back), short side = width (left/right).
    """
    x, y = center
    L = length / 2.0
    W = width  / 2.0
    # axis-aligned corners before rotation
    pts = [(x+L, y+W), (x+L, y-W), (x-L, y-W), (x-L, y+W)]
    # rotate all corners about (x, y)
    c, s = math.cos(theta), math.sin(theta)
    rot = []
    for px, py in pts:
        dx, dy = px - x, py - y
        rx = x + c*dx - s*dy
        ry = y + s*dx + c*dy
        rot.append((rx, ry))
    return rot

