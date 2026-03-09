from dataclasses import dataclass
import numpy as np

@dataclass
class Obstacle:
    """A circular obstacle in the environment."""
    x: float
    y: float
    radius: float

    def is_collision(self, px: float, py: float) -> bool:
        """Checks if a point (px, py) is inside the obstacle's radius."""
        return np.sqrt((px - self.x)**2 + (py - self.y)**2) <= self.radius
