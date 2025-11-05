# vehicles/base.py
from dataclasses import dataclass

@dataclass
class State2D:
    """Simple pose container (meters, radians)."""
    x: float
    y: float
    theta: float

