import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import time
import os
from rrt_planner import RRTStar

def run_visualization():
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
    
    output_filename = "warehouse_path_plan.png"
    print(f"Generating visualization: {output_filename}...")
    
    # Force Agg backend to ensure it works in all environments
    plt.switch_backend('Agg')
    
    start_time = time.time()
    path = rrt_star.plan(animate=True, save_path=output_filename)
    end_time = time.time()
    
    if path:
        print(f"Visualization generated in {end_time - start_time:.2f} seconds.")
        print(f"Path saved to {os.path.abspath(output_filename)}")
    else:
        print("Pathfinding Failed. Visualization could not be completed.")

if __name__ == "__main__":
    run_visualization()
