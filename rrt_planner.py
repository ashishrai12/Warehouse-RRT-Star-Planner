import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import time
import os

# Use a non-GUI backend if no display is detected (e.g., inside Docker)
if 'DISPLAY' not in os.environ and os.name != 'nt':
    plt.switch_backend('Agg')

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    """
    Rapidly-exploring Random Tree Star (RRT*) path planning algorithm.
    This implementation solves the optimal path planning problem in a 2D space with obstacles.
    """
    def __init__(self, start_pos, goal_pos, obstacles, area_bounds, step_size=0.5, max_iter=2000, search_radius=1.0):
        """
        :param start_pos: Tuple (x, y) for the initial position.
        :param goal_pos: Tuple (x, y) for the target position.
        :param obstacles: List of tuples (x, y, radius) representing circular obstacles.
        :param area_bounds: Tuple (xmin, xmax, ymin, ymax) for the configuration space.
        :param step_size: Distance to move from nearest node toward random sample.
        :param max_iter: Maximum number of iterations (samples).
        :param search_radius: Base radius for RRT* rewiring logic.
        """
        self.start = Node(start_pos[0], start_pos[1])
        self.goal = Node(goal_pos[0], goal_pos[1])
        self.obstacles = obstacles
        self.bounds = area_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.nodes = [self.start]
        self.path = []

    def plan(self, animate=True, save_path=None):
        """Main execution loop for the RRT* algorithm."""
        if animate:
            plt.ion()
            fig, ax = plt.subplots(figsize=(10, 10))
            self._draw_static_map(ax)

        for i in range(self.max_iter):
            # Sample random node with goal bias
            if random.random() < 0.1:
                rnd_node = Node(self.goal.x, self.goal.y)
            else:
                rnd_node = Node(random.uniform(self.bounds[0], self.bounds[1]), 
                                random.uniform(self.bounds[2], self.bounds[3]))

            # Find nearest node in existing tree
            nearest_node = self._get_nearest_node(rnd_node)
            
            # Steer from nearest node towards sample
            new_node = self._steer(nearest_node, rnd_node)

            # Check if path is collision-free
            if self._is_collision_free(nearest_node, new_node):
                # RRT* Optimization: Search for near neighbors
                near_nodes = self._find_near_nodes(new_node)
                
                # Connect along a path that minimizes cost
                best_node = nearest_node
                min_cost = nearest_node.cost + self._distance(nearest_node, new_node)
                
                for node in near_nodes:
                    if self._is_collision_free(node, new_node):
                        cost = node.cost + self._distance(node, new_node)
                        if cost < min_cost:
                            best_node = node
                            min_cost = cost
                
                new_node.parent = best_node
                new_node.cost = min_cost
                self.nodes.append(new_node)

                # RRT* Rewiring: Optimize neighbors by routing through new_node
                for node in near_nodes:
                    if self._is_collision_free(new_node, node):
                        cost = new_node.cost + self._distance(new_node, node)
                        if cost < node.cost:
                            node.parent = new_node
                            node.cost = cost

                if animate and i % 50 == 0:
                    self._draw_dynamic_tree(ax, new_node)
                    plt.pause(0.001)

                # Update goal parent if a shorter path is found
                if self._distance(new_node, self.goal) < self.step_size:
                    if self._is_collision_free(new_node, self.goal):
                        potential_goal_cost = new_node.cost + self._distance(new_node, self.goal)
                        if self.goal.parent is None or potential_goal_cost < self.goal.cost:
                            self.goal.parent = new_node
                            self.goal.cost = potential_goal_cost
            
        self.path = self._trace_path()
        
        if animate:
            plt.ioff()
            if self.path:
                self._draw_path(ax)
            if save_path:
                plt.savefig(save_path)
                print(f"Simulation plot saved as {save_path}")
            else:
                plt.show()
        
        return self.path

    def _get_nearest_node(self, rnd_node):
        dists = [self._distance(node, rnd_node) for node in self.nodes]
        return self.nodes[np.argmin(dists)]

    def _steer(self, from_node, to_node):
        dist = self._distance(from_node, to_node)
        if dist <= self.step_size:
            return Node(to_node.x, to_node.y)
        else:
            angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
            return Node(from_node.x + self.step_size * np.cos(angle), 
                        from_node.y + self.step_size * np.sin(angle))

    def _is_collision_free(self, from_node, to_node):
        """Samples the line segment for collisions with circle obstacles."""
        num_samples = int(self._distance(from_node, to_node) / 0.1) + 2
        for i in range(num_samples):
            t = i / (num_samples - 1)
            x = from_node.x * (1 - t) + to_node.x * t
            y = from_node.y * (1 - t) + to_node.y * t
            for (ox, oy, orad) in self.obstacles:
                if np.sqrt((x - ox)**2 + (y - oy)**2) <= orad:
                    return False
        return True

    def _find_near_nodes(self, new_node):
        """Finds nodes within a dynamic radius of the new node for rewiring."""
        n = len(self.nodes)
        radius = self.search_radius * np.sqrt(np.log(n) / n) if n > 1 else self.search_radius
        dists = [self._distance(node, new_node) for node in self.nodes]
        return [self.nodes[i] for i, d in enumerate(dists) if d <= radius]

    def _distance(self, n1, n2):
        return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def _trace_path(self):
        """Traces the path from goal back to start once search is complete."""
        if self.goal.parent is None:
            return []
        
        curr = self.goal
        path = []
        while curr is not None:
            path.append((curr.x, curr.y))
            curr = curr.parent
        return path[::-1]

    def _draw_static_map(self, ax):
        ax.set_xlim(self.bounds[0], self.bounds[1])
        ax.set_ylim(self.bounds[2], self.bounds[3])
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.5)
        
        for ox, oy, orad in self.obstacles:
            circle = patches.Circle((ox, oy), orad, color='gray', alpha=0.5)
            ax.add_patch(circle)
            
        ax.plot(self.start.x, self.start.y, "ro", markersize=8, label="Start (Sorting Station)")
        ax.plot(self.goal.x, self.goal.y, "go", markersize=8, label="Goal (Delivery Bay)")
        ax.set_title("Automated Warehouse: RRT* Motion Planning")
        ax.legend()

    def _draw_dynamic_tree(self, ax, node):
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color="cyan", alpha=0.3, linewidth=1)

    def _draw_path(self, ax):
        px = [p[0] for p in self.path]
        py = [p[1] for p in self.path]
        ax.plot(px, py, "r-", linewidth=3, label="Optimized Robot Path")
        ax.legend()

if __name__ == "__main__":
    # Define warehouse simulation parameters
    bounds = (0, 10, 0, 10)
    start = (0.5, 0.5)
    goal = (9.5, 9.5)
    
    # Simulate shelving units and obstacles in a warehouse
    warehouse_obstacles = [
        # Shelving Unit 1
        (2, 2, 0.8), (2, 3.5, 0.8), (2, 5, 0.8),
        # Shelving Unit 2
        (6, 7, 1.2), (7.5, 7, 1.2),
        # Processing equipment
        (4, 5, 1.0), (8, 3, 1.5),
        # Pillars
        (5, 5, 0.3), (5, 2, 0.3), (5, 8, 0.3)
    ]
    
    rrt_star = RRTStar(
        start, goal, warehouse_obstacles, bounds, 
        step_size=0.4, max_iter=2000, search_radius=1.5
    )
    
    print("Executing RRT* Path Planning for Automated Warehouse...")
    start_time = time.time()
    
    # If running headlessly (like in Docker), save to file
    is_headless = 'DISPLAY' not in os.environ and os.name != 'nt'
    path = rrt_star.plan(animate=True, save_path="warehouse_path_plan.png" if is_headless else None)
    
    end_time = time.time()
    
    if path:
        print(f"Path Successfully Found in {end_time - start_time:.2f} seconds.")
        print(f"Goal reached! Path contains {len(path)} nodes.")
    else:
        print("Pathfinding Failed. Increase max_iter or check start/goal validity.")
