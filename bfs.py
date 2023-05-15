import queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# import CMap2D
import json

import numpy as np
import time


# def generate_grid_map(points, cell_size, margin):
#     # Calculate the bounds of the point cloud
#     min_x, min_y = np.min(points, axis=0)
#     max_x, max_y = np.max(points, axis=0)

#     # Calculate the dimensions of the grid map
#     grid_width = int(np.ceil((max_x - min_x) / cell_size)) + margin
#     grid_height = int(np.ceil((max_y - min_y) / cell_size)) + margin

#     # Create an empty grid map
#     grid_map = np.zeros((grid_height, grid_width), dtype=np.int8)
#     # grid_map = [[0, gr]
#     # Convert the point coordinates to grid cell indices and mark them as occupied
#     for x, y in points:
#         grid_x = int(np.floor((x - min_x) / cell_size))
#         grid_y = int(np.floor((y - min_y) / cell_size))
#         grid_map[grid_y, grid_x] = 1

#     return grid_map


def generate_grid_map(points, resolution, margin):
    # Find the minimum and maximum x and y coordinates in the list of points
    min_x = min(points, key=lambda p: p[0])[0]
    max_x = max(points, key=lambda p: p[0])[0]
    min_y = min(points, key=lambda p: p[1])[1]
    max_y = max(points, key=lambda p: p[1])[1]

    # Add the margin to the boundaries of the grid
    min_x -= margin
    max_x += margin
    min_y -= margin
    max_y += margin

    # Calculate the width and height of the grid map
    width = int((max_x - min_x) / resolution) + 1
    height = int((max_y - min_y) / resolution) + 1

    # Initialize the grid map with all zeros
    grid_map = [[0 for j in range(width)] for i in range(height)]

    # Convert each point to a cell index in the grid map and set it to 1
    for point in points:
        x = int((point[0] - min_x) / resolution)
        y = int((point[1] - min_y) / resolution)
        grid_map[y][x] = 1

    return grid_map


def bfs_occupancy_grid_animation(grid, start, goal):
    fig, ax = plt.subplots()
    ax.set_xticks([i for i in range(len(grid[0]))])
    ax.set_yticks([i for i in range(len(grid))])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.imshow(grid, cmap='Greys', interpolation='nearest',
              vmin=0, vmax=1, aspect='equal')
    ax.plot(start[1], start[0], 'go', markersize=20)
    ax.plot(goal[1], goal[0], 'ro', markersize=20)

    # Define the 4-connected neighbours
    neighbours = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    axtext = fig.add_axes([0.0, 0.95, 0.1, 0.05])
    time_txt = axtext.text(0.5, 0.5, str(0), ha="left", va="top")
    now = time.time()
    def animate(i):
        nonlocal ax, grid
        # Initialize queue for BFS
        q = queue.Queue()
        # Mark start cell as visited and add it to the queue
        visited = set([start])
        q.put(start)

        while not q.empty():
            # Get the next cell from the queue
            current_cell = q.get()

            # Check if current cell is the goal
            # if current_cell == goal:
            #     return True

            # Explore neighbours of the current cell
            for neighbour in neighbours:
                neighbour_cell = (
                    current_cell[0] + neighbour[0], current_cell[1] + neighbour[1])

                # Check if neighbour cell is within the grid bounds and unvisited
                if (0 <= neighbour_cell[0] < len(grid) and
                    0 <= neighbour_cell[1] < len(grid[0]) and
                    neighbour_cell not in visited and
                        grid[neighbour_cell[0]][neighbour_cell[1]] != 1):  # Check for obstacle (assuming 1 indicates an obstacle)

                    # Mark neighbour cell as visited and add it to the queue
                    visited.add(neighbour_cell)
                    q.put(neighbour_cell)
                    # Update grid visualization...
                    later = time.time()
                    difference = int(later - now)
                    time_txt.set_text(difference)
                    grid[neighbour_cell[0]][neighbour_cell[1]] = 0.5
                    ax.imshow(grid, cmap='Greys', interpolation='nearest',
                              vmin=0, vmax=1, aspect='equal')
                    plt.pause(0.001)

        # Goal not found
        return False

    ani = animation.FuncAnimation(fig, animate, frames=None, repeat=False)
    plt.show()

    return ani


# empty map
# mymap = CMap2D()
with open('input_data.json', 'r') as f:
        data = json.load(f)

if not data:
    print("Invalid JSON")
    exit()

x_points = []
y_points = []
points = []

for i in range(len(data['uav1'][0])):
    points.append([data['uav1'][0][i][0][0], data['uav1'][0][i][0][1]])
    x_points.append(data['uav1'][0][i][0][0])
    y_points.append(data['uav1'][0][i][0][1])
# print(points)
# # from an array

# mymap.from_array(points, [0,0], 0.25)


# # Example occupancy grid
# grid = [
#     [0, 0, 0, 0, 0],
#     [0, 1, 0, 1, 0],
#     [0, 0, 0, 0, 0],
#     [1, 1, 1, 0, 0],
#     [0, 0, 0, 0, 0]
# ]
grid = generate_grid_map(points, 1, 1)
start = (int((len(grid)-1)/2), int((len(grid[0])-1)/2))
# start = (int(len(grid)-1)/2, int(len(grid[0])-1)/2)
# goal = (1,1)
# goal = grid.shape
goal = (len(grid)-1, len(grid[0])-1)
# print(grid)

# # Start and goal positions

# Animate the path using BFS
bfs_occupancy_grid_animation(grid, start, goal)
