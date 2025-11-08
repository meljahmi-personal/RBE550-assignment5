# plan/neighbors.py
from typing import Iterator, Tuple

def neighbors4(x: int, y: int) -> Iterator[Tuple[int, int, float]]:
    yield x + 1, y, 1.0
    yield x - 1, y, 1.0
    yield x, y + 1, 1.0
    yield x, y - 1, 1.0

def neighbors8(x: int, y: int) -> Iterator[Tuple[int, int, float]]:
    # 4-connected
    yield x + 1, y, 1.0
    yield x - 1, y, 1.0
    yield x, y + 1, 1.0
    yield x, y - 1, 1.0
    # diagonals (cost = sqrt(2) ≈ 1.4142)
    yield x + 1, y + 1, 1.4142135623730951
    yield x + 1, y - 1, 1.4142135623730951
    yield x - 1, y + 1, 1.4142135623730951
    yield x - 1, y - 1, 1.4142135623730951

