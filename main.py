import os
import time
from src.models import Obstacle, WarehouseConfig
from src.algorithms.rrt_star import RRTStar
from src.utils.visualizer import Visualizer

def main():
    bounds = (0, 10, 0, 10)
    start = (0.5, 0.5)
    goal = (9.5, 9.5)
    
    warehouse_obstacles = [
        Obstacle(2, 2, 0.8), Obstacle(2, 3.5, 0.8), Obstacle(2, 5, 0.8),
        Obstacle(6, 7, 1.2), Obstacle(7.5, 7, 1.2),
        Obstacle(4, 5, 1.0), Obstacle(8, 3, 1.5),
        Obstacle(5, 5, 0.3), Obstacle(5, 2, 0.3), Obstacle(5, 8, 0.3)
    ]
    
    config = WarehouseConfig(
        bounds=bounds,
        start=start,
        goal=goal,
        obstacles=warehouse_obstacles,
        step_size=0.4,
        max_iter=2000,
        search_radius=1.5
    )
    
    is_headless = 'DISPLAY' not in os.environ and os.name != 'nt'
    if is_headless:
        Visualizer.use_headless()
    
    visualizer = Visualizer(config)
    planner = RRTStar(config)
    
    print("Executing RRT* Path Planning for Automated Warehouse...")
    
    start_time = time.time()
    path = planner.plan(on_iteration=visualizer.update_tree)
    end_time = time.time()
    
    if path:
        print(f"Path Successfully Found in {end_time - start_time:.2f} seconds.")
        print(f"Goal reached! Path contains {len(path)} nodes.")
        
        visualizer.draw_path(path)
        
        if is_headless:
            visualizer.save("warehouse_path_plan.png")
        else:
            visualizer.show()
    else:
        print("Pathfinding Failed. Increase max_iter or check start/goal validity.")

if __name__ == "__main__":
    main()
