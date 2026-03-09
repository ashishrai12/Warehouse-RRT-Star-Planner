from dataclasses import dataclass, field
from typing import Optional

@dataclass
class Node:
    """A point in the 2D search space, with its cost and parent trace."""
    x: float
    y: float
    parent: Optional['Node'] = None
    cost: float = 0.0
