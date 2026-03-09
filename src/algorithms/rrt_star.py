import numpy as np
import random
from typing import List, Optional, Tuple, Callable
from ..models import Node, WarehouseConfig

class RRTStar:
    """Core RRT* Algorithm Implementation."""
    def __init__(self, config: WarehouseConfig):
        self.config = config
        self.start_node = Node(config.start[0], config.start[1])
        self.goal_node = Node(config.goal[0], config.goal[1])
        self.nodes = [self.start_node]
        self.path = []

    def plan(self, on_iteration: Optional[Callable[[Node], None]] = None) -> List[Tuple[float, float]]:
        """Executes the RRT* algorithm and returns the path."""
        for i in range(self.config.max_iter):
            rnd_node = self._sample()
            nearest_node = self._nearest(rnd_node)
            new_node = self._steer(nearest_node, rnd_node)

            if self._is_collision_free(nearest_node, new_node):
                near_nodes = self._find_near_nodes(new_node)
                
                best_parent = nearest_node
                min_cost = nearest_node.cost + self._distance(nearest_node, new_node)
                
                for near_node in near_nodes:
                    if self._is_collision_free(near_node, new_node):
                        cost = near_node.cost + self._distance(near_node, new_node)
                        if cost < min_cost:
                            best_parent = near_node
                            min_cost = cost
                
                new_node.parent = best_parent
                new_node.cost = min_cost
                self.nodes.append(new_node)

                for near_node in near_nodes:
                    if self._is_collision_free(new_node, near_node):
                        cost = new_node.cost + self._distance(new_node, near_node)
                        if cost < near_node.cost:
                            near_node.parent = new_node
                            near_node.cost = cost

                if on_iteration:
                    on_iteration(new_node)

                if self._distance(new_node, self.goal_node) < self.config.step_size:
                    if self._is_collision_free(new_node, self.goal_node):
                        potential_cost = new_node.cost + self._distance(new_node, self.goal_node)
                        if self.goal_node.parent is None or potential_cost < self.goal_node.cost:
                            self.goal_node.parent = new_node
                            self.goal_node.cost = potential_cost
        
        self.path = self._trace_path()
        return self.path

    def _sample(self) -> Node:
        if random.random() < self.config.goal_bias:
            return Node(self.goal_node.x, self.goal_node.y)
        return Node(
            random.uniform(self.config.bounds[0], self.config.bounds[1]),
            random.uniform(self.config.bounds[2], self.config.bounds[3])
        )

    def _nearest(self, rnd_node: Node) -> Node:
        dists = [self._distance(n, rnd_node) for n in self.nodes]
        return self.nodes[np.argmin(dists)]

    def _steer(self, from_node: Node, to_node: Node) -> Node:
        dist = self._distance(from_node, to_node)
        if dist <= self.config.step_size:
            return Node(to_node.x, to_node.y)
        angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return Node(
            from_node.x + self.config.step_size * np.cos(angle),
            from_node.y + self.config.step_size * np.sin(angle)
        )

    def _is_collision_free(self, n1: Node, n2: Node) -> bool:
        steps = int(self._distance(n1, n2) / 0.1) + 2
        for i in range(steps):
            t = i / (steps - 1)
            px = n1.x * (1 - t) + n2.x * t
            py = n1.y * (1 - t) + n2.y * t
            for obs in self.config.obstacles:
                if obs.is_collision(px, py):
                    return False
        return True

    def _find_near_nodes(self, node: Node) -> List[Node]:
        n = len(self.nodes)
        radius = self.config.search_radius * np.sqrt(np.log(n) / n) if n > 1 else self.config.search_radius
        return [node_i for node_i in self.nodes if self._distance(node_i, node) <= radius]

    def _distance(self, n1: Node, n2: Node) -> float:
        return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def _trace_path(self) -> List[Tuple[float, float]]:
        if not self.goal_node.parent:
            return []
        path = []
        curr = self.goal_node
        while curr:
            path.append((curr.x, curr.y))
            curr = curr.parent
        return path[::-1]
