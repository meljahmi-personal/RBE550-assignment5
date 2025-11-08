# plan/base.py
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class PlanResult:
    success: bool
    path: List[Tuple[int, int]]
    nodes_expanded: int
    peak_open: int
    expanded: int

class BasePlanner:
    name: str = "base"

