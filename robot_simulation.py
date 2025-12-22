import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# --- Configuration ---
GRID_SIZE = 20
NUM_ROBOTS = 3
NUM_ITEMS = 10
OBSTACLE_DENSITY = 0.2

# --- States ---
STATE_IDLE = "IDLE"
STATE_MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
STATE_PICKING = "PICKING"
STATE_MOVING_TO_DROP = "MOVING_TO_DROP"
STATE_DROPPING = "DROPPING"
STATE_BROKEN = "BROKEN"

# --- Colors ---
COLOR_EMPTY = 'white'
COLOR_OBSTACLE = 'black'
COLOR_ROBOT = 'blue'
COLOR_ROBOT_BROKEN = 'red'
COLOR_ITEM_URGENT = 'orange'
COLOR_ITEM_REGULAR = 'green'
COLOR_PATH = 'yellow'

class Item:
    def __init__(self, item_id, priority, pickup_loc, drop_loc):
        self.id = item_id
        self.priority = priority  # 2: Urgent, 1: Regular
        self.pickup_loc = pickup_loc
        self.drop_loc = drop_loc
        self.status = 'waiting' # waiting, assigned, picked, delivered

class Robot:
    def __init__(self, robot_id, start_pos, warehouse):
        self.id = robot_id
        self.pos = start_pos
        self.warehouse = warehouse
        self.state = STATE_IDLE
        self.battery = 100.0
        self.current_task = None
        self.path = []
        self.target_pos = None

    def assign_task(self, task):
        self.current_task = task
        self.state = STATE_MOVING_TO_PICKUP
        self.target_pos = task.pickup_loc
        self.calculate_path()

    def calculate_path(self):
        if self.target_pos:
            self.path = self.warehouse.find_path(self.pos, self.target_pos)

    def move(self):
        if self.state == STATE_BROKEN:
            return

        if self.battery <= 0:
            self.state = STATE_BROKEN
            return

        if self.path:
            next_pos = self.path[0]
            
            # Dynamic Obstacle Check: Check if another robot is at next_pos
            if not self.warehouse.is_occupied_by_robot(next_pos, self.id):
                self.pos = self.path.pop(0)
                self.battery -= 0.1
            else:
                # Wait or re-route (simple wait for now)
                pass
        else:
            # Path is empty, but we haven't reached target?
            if self.pos != self.target_pos and self.target_pos is not None:
                self.calculate_path()

    def update(self):
        # Random Failure Simulation
        if self.state != STATE_BROKEN and random.random() < 0.005: # 0.5% chance per frame
            self.state = STATE_BROKEN
            print(f"Robot {self.id} experienced a failure!")
            return

        if self.state == STATE_BROKEN:
            # Simple self-repair logic for simulation continuity
            if random.random() < 0.02:
                self.state = STATE_IDLE
                self.battery = 100
                print(f"Robot {self.id} repaired.")
            return

        if self.state == STATE_IDLE:
            return

        # State Machine Logic
        if self.state == STATE_MOVING_TO_PICKUP:
            if self.pos == self.target_pos:
                self.state = STATE_PICKING
            else:
                self.move()
        
        elif self.state == STATE_PICKING:
            # Simulate picking action (instant for now, could add delay)
            if self.current_task:
                self.current_task.status = 'picked'
            self.state = STATE_MOVING_TO_DROP
            if self.current_task:
                self.target_pos = self.current_task.drop_loc
            self.calculate_path()

        elif self.state == STATE_MOVING_TO_DROP:
            if self.pos == self.target_pos:
                self.state = STATE_DROPPING
            else:
                self.move()

        elif self.state == STATE_DROPPING:
            if self.current_task:
                self.current_task.status = 'delivered'
                self.current_task = None
            self.state = STATE_IDLE
            self.target_pos = None

class Warehouse:
    def __init__(self, size):
        self.size = size
        self.grid = np.zeros((size, size)) # 0: Empty, 1: Obstacle
        self.robots = []
        self.generate_layout()

    def generate_layout(self):
        # Create aisles and shelves
        # Vertical shelves
        for x in range(2, self.size - 2, 3):
            self.grid[2:self.size-2, x:x+2] = 1 
        
        # Loading zones (keep clear)
        self.grid[0:2, :] = 0
        self.grid[self.size-2:self.size, :] = 0

    def is_valid(self, pos):
        r, c = pos
        return 0 <= r < self.size and 0 <= c < self.size and self.grid[r, c] == 0

    def is_occupied_by_robot(self, pos, my_id):
        for robot in self.robots:
            if robot.id != my_id and robot.pos == pos:
                return True
        return False

    def find_path(self, start, end):
        # A* Algorithm
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, end)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if current == end:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                if not self.is_valid(neighbor):
                    continue
                
                tentative_g_score = gscore[current] + 1

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        return [] # No path found

class Simulation:
    def __init__(self):
        self.warehouse = Warehouse(GRID_SIZE)
        self.tasks = []
        self.setup_robots()
        self.setup_tasks()
        
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.anim = animation.FuncAnimation(self.fig, self.update, frames=200, interval=200, repeat=True)

    def setup_robots(self):
        for i in range(NUM_ROBOTS):
            # Start robots at the top loading zone
            start_pos = (0, i * 2) 
            self.warehouse.robots.append(Robot(i, start_pos, self.warehouse))

    def setup_tasks(self):
        for i in range(NUM_ITEMS):
            # Random pickup from shelves area
            while True:
                pr, pc = random.randint(2, GRID_SIZE-3), random.randint(0, GRID_SIZE-1)
                if self.warehouse.is_valid((pr, pc)):
                    pickup = (pr, pc)
                    break
            
            # Drop off at bottom loading zone
            drop = (GRID_SIZE-1, random.randint(0, GRID_SIZE-1))
            
            priority = random.choice([1, 2]) # 2 is urgent
            self.tasks.append(Item(i, priority, pickup, drop))
        
        # Sort tasks by priority (Urgent first)
        self.tasks.sort(key=lambda x: x.priority, reverse=True)

    def assign_tasks(self):
        waiting_tasks = [t for t in self.tasks if t.status == 'waiting']
        
        for robot in self.warehouse.robots:
            if robot.state == STATE_IDLE and waiting_tasks:
                task = waiting_tasks.pop(0)
                task.status = 'assigned'
                robot.assign_task(task)

    def update(self, frame):
        self.assign_tasks()
        for robot in self.warehouse.robots:
            robot.update()
        
        self.draw()

    def draw(self):
        self.ax.clear()
        
        # Draw Grid (0 is white, 1 is black)
        # We use a custom cmap to make obstacles look like shelves
        self.ax.imshow(self.warehouse.grid, cmap='binary', origin='upper', vmin=0, vmax=1)
        
        # Draw Items
        for item in self.tasks:
            if item.status == 'waiting':
                color = COLOR_ITEM_URGENT if item.priority == 2 else COLOR_ITEM_REGULAR
                self.ax.scatter(item.pickup_loc[1], item.pickup_loc[0], c=color, marker='s', s=60, label='Item' if item.id == 0 else "")
            elif item.status == 'delivered':
                 self.ax.scatter(item.drop_loc[1], item.drop_loc[0], c='gray', marker='x', s=40, alpha=0.5)

        # Draw Robots and Paths
        for robot in self.warehouse.robots:
            color = COLOR_ROBOT_BROKEN if robot.state == STATE_BROKEN else COLOR_ROBOT
            
            # Draw Path
            if robot.path:
                path_y = [p[0] for p in robot.path]
                path_x = [p[1] for p in robot.path]
                self.ax.plot(path_x, path_y, c=COLOR_PATH, alpha=0.6, linewidth=2)

            # Draw Robot
            self.ax.scatter(robot.pos[1], robot.pos[0], c=color, s=150, marker='o', edgecolors='black', zorder=10)
            self.ax.text(robot.pos[1], robot.pos[0], str(robot.id), color='white', ha='center', va='center', fontweight='bold')

            # Draw carried item
            if robot.state in [STATE_PICKING, STATE_MOVING_TO_DROP] and robot.current_task:
                self.ax.scatter(robot.pos[1], robot.pos[0], c='yellow', marker='*', s=50, zorder=11)

        # Legend and Status
        status_text = f"Robots: {len(self.warehouse.robots)} | Tasks Pending: {len([t for t in self.tasks if t.status == 'waiting'])}"
        self.ax.set_title(f"Warehouse Simulation\n{status_text}")
        
        # Create custom legend handles
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_ROBOT, label='Robot', markersize=10),
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_ROBOT_BROKEN, label='Broken Robot', markersize=10),
            Line2D([0], [0], marker='s', color='w', markerfacecolor=COLOR_ITEM_URGENT, label='Urgent Item', markersize=8),
            Line2D([0], [0], marker='s', color='w', markerfacecolor=COLOR_ITEM_REGULAR, label='Regular Item', markersize=8),
            Line2D([0], [0], color=COLOR_PATH, lw=2, label='Path'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.15, 1))

if __name__ == "__main__":
    sim = Simulation()
    plt.show()
