import unittest
import numpy as np
from src.models import Node, Obstacle, WarehouseConfig
from src.algorithms.rrt_star import RRTStar

class TestRRTStar(unittest.TestCase):
    def setUp(self):
        self.config = WarehouseConfig(
            bounds=(0, 10, 0, 10),
            start=(1, 1),
            goal=(9, 9),
            obstacles=[], # No obstacles for simple testing
            max_iter=100
        )
        self.rrt = RRTStar(self.config)

    def test_node_init(self):
        node = Node(2.0, 3.0)
        self.assertEqual(node.x, 2.0)
        self.assertEqual(node.y, 3.0)
        self.assertIsNone(node.parent)
        self.assertEqual(node.cost, 0.0)

    def test_distance(self):
        n1 = Node(0, 0)
        n2 = Node(3, 4)
        self.assertEqual(self.rrt._distance(n1, n2), 5.0)

    def test_steer(self):
        n1 = Node(0, 0)
        n2 = Node(10, 0)
        self.config.step_size = 2.0
        n_new = self.rrt._steer(n1, n2)
        self.assertEqual(n_new.x, 2.0)
        self.assertEqual(n_new.y, 0.0)

        # Distance less than step_size
        n2_close = Node(1, 0)
        n_new_close = self.rrt._steer(n1, n2_close)
        self.assertEqual(n_new_close.x, 1.0)
        self.assertEqual(n_new_close.y, 0.0)

    def test_collision_free(self):
        # With no obstacles, any path should be collision free
        n1 = Node(0, 0)
        n2 = Node(5, 5)
        self.assertTrue(self.rrt._is_collision_free(n1, n2))

        # With an obstacle in between
        self.config.obstacles = [Obstacle(2.5, 2.5, 1.0)]
        self.assertFalse(self.rrt._is_collision_free(n1, n2))

        # Outside obstacle
        n3 = Node(0, 5)
        self.assertTrue(self.rrt._is_collision_free(n1, n3))

    def test_planning_success(self):
        # A simple straight line should work easily
        self.config.max_iter = 500
        path = self.rrt.plan()
        self.assertTrue(len(path) >= 2)
        self.assertEqual(path[0], self.config.start)
        # Check if the last point is close to the goal (within step_size)
        dist_to_goal = np.sqrt((path[-1][0] - self.config.goal[0])**2 + (path[-1][1] - self.config.goal[1])**2)
        self.assertLess(dist_to_goal, self.config.step_size + 0.1)

if __name__ == "__main__":
    unittest.main()
