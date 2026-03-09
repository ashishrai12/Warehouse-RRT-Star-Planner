from dataclasses import dataclass, field
from typing import List, Tuple
from .obstacle import Obstacle

@dataclass
class WarehouseConfig:
    """Parameters for the warehouse environment and planner."""
    bounds: Tuple[float, float, float, float]
    start: Tuple[float, float]
    goal: Tuple[float, float]
    obstacles: List[Obstacle] = field(default_factory=list)
    step_size: float = 0.4
    max_iter: int = 2000
    search_radius: float = 1.5
    goal_bias: float = 0.1
