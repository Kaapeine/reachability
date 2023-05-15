import pygame
import queue

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Define cell size and margin
CELL_SIZE = 20
MARGIN = 5

def draw_grid(grid, screen):
    # Clear the screen
    screen.fill(WHITE)
    
    # Draw the grid
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            cell_color = WHITE
            if grid[row][col] == 1:
                cell_color = BLACK
            pygame.draw.rect(screen, cell_color, [(MARGIN + CELL_SIZE) * col + MARGIN,
                                                  (MARGIN + CELL_SIZE) * row + MARGIN,
                                                  CELL_SIZE,
                                                  CELL_SIZE])
            
def draw_path(path, screen):
    # Draw the path
    for cell in path:
        pygame.draw.rect(screen, BLUE, [(MARGIN + CELL_SIZE) * cell[1] + MARGIN,
                                        (MARGIN + CELL_SIZE) * cell[0] + MARGIN,
                                        CELL_SIZE,
                                        CELL_SIZE])

def bfs_occupancy_grid(grid, start, goal, screen):
    # Initialize queue for BFS
    q = queue.Queue()
    # Mark start cell as visited and add it to the queue
    visited = set([start])
    q.put(start)
    
    # Define the 4-connected neighbours
    neighbours = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    
    while not q.empty():
        # Get the next cell from the queue
        current_cell = q.get()
        
        # Check if current cell is the goal
        if current_cell == goal:
            # Reconstruct path
            path = [current_cell]
            while current_cell != start:
                current_cell = parents[current_cell]
                path.append(current_cell)
            path.reverse()
            # Draw the path and return it
            draw_path(path, screen)
            return path
        
        # Explore neighbours of the current cell
        for neighbour in neighbours:
            neighbour_cell = (current_cell[0] + neighbour[0], current_cell[1] + neighbour[1])
            
            # Check if neighbour cell is within the grid bounds and unvisited
            if (0 <= neighbour_cell[0] < len(grid) and
                0 <= neighbour_cell[1] < len(grid[0]) and
                neighbour_cell not in visited and
                grid[neighbour_cell[0]][neighbour_cell[1]] != 1): # Check for obstacle (assuming 1 indicates an obstacle)
                
                # Mark neighbour cell as visited, add it to the queue, and set its parent
                visited.add(neighbour_cell)
                q.put(neighbour_cell)
                parents[neighbour_cell] = current_cell
                
                # Draw the current cell and neighbour cell
                pygame.draw.rect(screen, RED, [(MARGIN + CELL_SIZE) * current_cell[1] + MARGIN,
                                               (MARGIN + CELL_SIZE) * current_cell[0] + MARGIN,
                                               CELL_SIZE,
                                               CELL_SIZE])
                pygame.draw.rect(screen, GREEN, [(MARGIN + CELL_SIZE) * neighbour_cell[1] + MARGIN,
                                                 (MARGIN + CELL_SIZE) * neighbour_cell[0] + MARGIN,
                                                 CELL_SIZE,
                                                 CELL_SIZE
