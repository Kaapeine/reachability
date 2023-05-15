import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Define the movements allowed (4-directional)
movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]

def bfs_search(grid_map, start, goal):
    # Initialize the queue with the starting position
    queue = deque([start])
    visited = set([start])
    paths = {start: []}

    while queue:
        current_pos = queue.popleft()

        if current_pos == goal:
            return paths[current_pos]

        for move in movements:
            next_pos = (current_pos[0] + move[0], current_pos[1] + move[1])

            if (0 <= next_pos[0] < len(grid_map) and 
                0 <= next_pos[1] < len(grid_map[0]) and 
                grid_map[next_pos[0]][next_pos[1]] == 0 and 
                next_pos not in visited):
                
                queue.append(next_pos)
                visited.add(next_pos)
                paths[next_pos] = paths[current_pos] + [current_pos]

    return None

# Example 2D grid map
grid_map = np.array([
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
])

# Starting position
start = (0, 0)

# Goal position
goal = (4, 4)

# Find the shortest path from start to goal using BFS search
shortest_path = bfs_search(grid_map, start, goal)

# Initialize the figure and axis
fig, ax = plt.subplots()

# Set the axis limits
ax.set_xlim(-0.5, len(grid_map[0]) - 0.5)
ax.set_ylim(-0.5, len(grid_map) - 0.5)

# Initialize the grid map plot
grid_plot = ax.imshow(grid_map, cmap='Greys', interpolation='none')

# Initialize the current position plot
current_pos_plot = ax.scatter(start[1], start[0], marker='o', color='red')

# Initialize the path plot
path_plot, = ax.plot([], [], linewidth=2, color='blue')

# Define the animation function
def animate(frame):
    # Get the current position of the path
    current_pos = shortest_path[frame]

    # Update the current position plot
    current_pos_plot.set_offsets(current_pos)

    # Update the path plot
    path_plot.set_data([pos[1] for pos in shortest_path[:frame+1]],
                       [pos[0] for pos in shortest_path[:frame+1]])

    # Return the updated plots
    return (current_pos_plot, path_plot)

# Create the animation
animation = FuncAnimation(fig, animate, frames=len(shortest_path), interval=500)

# Show the animation
plt.show()
