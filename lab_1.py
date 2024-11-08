import heapq  # For priority queue in A* algorithm
import numpy as np  # For creating the grid and managing grid operations

def create_grid(size, obstacles, target_pos):
    """
    Creates a grid with obstacles and a target.
    :param size: Tuple for grid dimensions (rows, cols)
    :param obstacles: List of obstacle coordinates [(x1, y1), (x2, y2), ...]
    :param target_pos: Position of the target (x, y)
    :return: 2D numpy array representing the grid
    """
    grid = np.zeros(size, dtype=int)  # Initialize grid with open spaces
    for obs in obstacles:
        grid[obs] = 1  # Place obstacles
    grid[target_pos] = 2  # Set target
    return grid

class Agent:
    def __init__(self, start_pos):
        self.position = start_pos  # Initial position of the agent
        self.path = []  # To store the path found by A*

    def move(self, new_position):
        """Updates the agent's position."""
        self.position = new_position
        self.path.append(new_position)

def heuristic(a, b):
    """Heuristic function for A*: Manhattan distance"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}
    
    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_pos = (x + dx, y + dy)

            if 0 <= next_pos[0] < rows and 0 <= next_pos[1] < cols and grid[next_pos] != 1:
                new_cost = cost_so_far[current] + 1

                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    heapq.heappush(open_list, (priority, next_pos))
                    came_from[next_pos] = current

    return []  # Return empty path if goal not reachable

# Grid parameters
grid_size = (10, 10)
start_pos = (0, 0)
target_pos = (8, 9)

obstacles = [(1, 2), (2, 2), (3, 2), (4, 4), (5, 5), (6, 7), (7, 7), (8, 3)]

# Create grid and initialize agent
grid = create_grid(grid_size, obstacles, target_pos)
agent = Agent(start_pos)

# Run A* pathfinding
path = a_star_search(grid, start_pos, target_pos)
if path:
    for step in path:
        agent.move(step)
        print(f"Agent moved to {step}")
else:
    print("No path found to target!")

# Run the A* search to find the path to the target
print("Path to target:", path)