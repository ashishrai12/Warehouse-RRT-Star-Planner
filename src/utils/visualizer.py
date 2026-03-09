import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
from typing import List, Tuple, Optional
from ..models import Node, WarehouseConfig

class Visualizer:
    """Visualization utilities for the RRT* planner."""
    def __init__(self, config: WarehouseConfig):
        self.config = config
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self._setup_plot()

    def _setup_plot(self):
        self.ax.set_xlim(self.config.bounds[0], self.config.bounds[1])
        self.ax.set_ylim(self.config.bounds[2], self.config.bounds[3])
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        
        for obs in self.config.obstacles:
            circle = patches.Circle((obs.x, obs.y), obs.radius, color='gray', alpha=0.5)
            self.ax.add_patch(circle)
            
        self.ax.plot(self.config.start[0], self.config.start[1], "ro", markersize=8, label="Start (Sorting Station)")
        self.ax.plot(self.config.goal[0], self.config.goal[1], "go", markersize=8, label="Goal (Delivery Bay)")
        self.ax.set_title("Automated Warehouse: RRT* Motion Planning")
        self.ax.legend()

    def update_tree(self, node: Node):
        if node.parent:
            self.ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color="cyan", alpha=0.3, linewidth=1)

    def draw_path(self, path: List[Tuple[float, float]]):
        if path:
            px, py = zip(*path)
            self.ax.plot(px, py, "r-", linewidth=3, label="Optimized Robot Path")
            self.ax.legend()

    def save(self, filename: str):
        self.ax.legend()
        self.fig.savefig(filename)
        print(f"Simulation plot saved as {filename}")

    @staticmethod
    def show():
        plt.show()

    @staticmethod
    def use_headless():
        import matplotlib
        matplotlib.use('Agg')
