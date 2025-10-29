# env/tetromino.py
import random

# 7 classic tetrominoes as (x,y) cells with origin at (0,0)
_TETS = [
    [(0,0),(0,1),(0,2),(0,3)],            # I
    [(0,0),(0,1),(1,0),(1,1)],            # O
    [(0,0),(0,1),(0,2),(1,1)],            # T
    [(0,0),(1,0),(2,0),(2,1)],            # L
    [(0,1),(1,1),(2,1),(2,0)],            # J
    [(0,1),(0,2),(1,0),(1,1)],            # S
    [(0,0),(0,1),(1,1),(1,2)],            # Z
]

def _rot(shape, k):
    """Rotate shape by k*90°; then shift so all coords are non-negative."""
    pts = shape
    for _ in range(k % 4):
        pts = [(-y, x) for (x, y) in pts]
    minx = min(x for x,_ in pts)
    miny = min(y for _,y in pts)
    return [(x - minx, y - miny) for (x,y) in pts]

def place_tetrominoes(n, target_occupancy=0.10, seed=0):
    """
    Return a set of occupied grid cells (i,j) for an n-by-n grid using tetrominoes.
    target_occupancy ~ fraction of cells (e.g., 0.10 ≈ 10%).
    """
    rng = random.Random(seed)
    want = int(n*n * target_occupancy)
    occ = set()

    def can_place(cells):
        for (i,j) in cells:
            if not (0 <= i < n and 0 <= j < n): return False
            if (i,j) in occ: return False
        return True

    attempts = 0
    while len(occ) < want and attempts < 20000:
        attempts += 1
        i0 = rng.randrange(0, n)
        j0 = rng.randrange(0, n)
        shape = rng.choice(_TETS)
        rotk  = rng.randrange(0, 4)
        cells = [(i0 + x, j0 + y) for (x,y) in _rot(shape, rotk)]
        if can_place(cells):
            occ.update(cells)
    return occ

