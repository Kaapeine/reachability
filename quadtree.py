from pyqtree import Index
import quads
import numpy as np
import json
import math
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.collections import PatchCollection
from dijkstra import Dijkstra

stepAnimation = False

class Sector:
    def __init__(self, xmin, xmax, ymin, ymax):
        self.bb = [xmin, xmax, ymin,ymax]

        self.parent = None

        self.reachable = False
        self.empty = False

        self.color = 'white'

        self.center = [(xmax+xmin)/2, (ymax+ymin)/2]
        self.path = [] # Stores parent sector centers which were reachable
        self.last_reached = None
        self.size = xmax-xmin if xmax-xmin > ymax-ymin else ymax-ymin

def checkSector(current):
    if current.size <= drone_radius:
        return
    
    # check occupancy
    obstacles = len(tree.within_bb(quads.BoundingBox(min_x=current.bb[0], max_x=current.bb[1], min_y=current.bb[2], max_y=current.bb[3])))
    
    # if there are no obstacles and previous sector's center was reachable
    if obstacles == 0 and current.parent.last_reached == current.parent.center:
        print("Skip path searching")
        current.color = 'green'
        current.last_reached = current.center
        animate(current.color, current)
        return

    # find a path from last_reached to current sector's center
    sector = current
    while sector:
        if findPath(sector.last_reached[0], sector.last_reached[1], current.center[0], current.center[1]): # check if path exists, but ignoring the check right now, assume every sector is reachable
            current.last_reached = current.center
            break
        else:
            sector = sector.parent

    # if there are no obstacles and a path exists 
    if obstacles == 0 and current.last_reached == current.center:
        current.color = 'green'
        animate(current.color, current)
        return

    # make an animation only checking occupancy
    # so if a sector is empty and the its p
    
    # if empty and 
    if obstacles > 1:
        current.color = 'white'
        # More than 1 intersection means need to check child sectors as well
        child_size = current.size/2
        xmin = current.bb[0]
        xmax = current.bb[1]
        ymin = current.bb[2]
        ymax = current.bb[3]

        left_bottom = Sector(xmin, xmin+child_size, ymin, ymin+child_size)
        left_bottom.parent = current
        left_bottom.last_reached = current.last_reached
        left_top = Sector(xmin, xmin+child_size, ymin+child_size, ymax)
        left_top.parent = current
        left_top.last_reached = current.last_reached
        right_bottom = Sector(xmin+child_size, xmax, ymin, ymin+child_size)
        right_bottom.parent = current
        right_bottom.last_reached = current.last_reached
        right_top = Sector(xmin+child_size, xmax, ymin+child_size, ymax)
        right_top.parent = current
        right_top.last_reached = current.last_reached

        sector_queue.append(left_bottom)
        sector_queue.append(left_top)
        sector_queue.append(right_bottom)
        sector_queue.append(right_top)

    animate(current.color, current)

rectangles = []

def generateGridMap(points, resolution, margin):
    # Find the minimum and maximum x and y coordinates in the list of points
    # min_x = min(points, key=lambda p: p[0])[0]
    # max_x = max(points, key=lambda p: p[0])[0]
    # min_y = min(points, key=lambda p: p[1])[1]
    # max_y = max(points, key=lambda p: p[1])[1]

    # Set min and max to map_size
    min_x = -map_size/2
    min_y = -map_size/2
    max_x = map_size/2
    max_y = map_size/2

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

def findPath(sx, sy, gx, gy):
    global pathCount
    pathCount = pathCount + 1
    print(pathCount)
    # print(sx, sy, gx, gy)
    rx, ry = planner.planning(sx, sy, gx, gy)
    if rx == -1:
        # print("Not found path")
        return False
    # print("Found a path")
    return True

def animate(c, current):

    rectangles.append(plt.Rectangle((current.center[0]-current.size/2, current.center[1]-current.size/2), current.size, current.size, fill=True, facecolor=c, edgecolor='black'))
    if not stepAnimation:
        return
    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.add_patch(plt.Rectangle((-20, -20), 40, 40, fill=True, facecolor='white'))
    
    # if current.parent:
    #     rectangles.append(plt.Rectangle((current.parent.center[0]-current.parent.size/2, current.parent.center[1]-current.parent.size/2), current.parent.size, current.parent.size
    #                             , fill=True, facecolor=current.parent.color, edgecolor='black'))
    # if current.parent:
    #     ax.add_patch(plt.Rectangle((current.parent.center[0]-current.parent.size/2, current.parent.center[1]-current.parent.size/2), current.parent.size, current.parent.size
    #                             , fill=True, facecolor=current.parent.color, edgecolor='black'))
    # ax.add_patch(plt.Rectangle((current.center[0]-current.size/2, current.center[1]-current.size/2), current.size, current.size, fill=True, facecolor=c, edgecolor='black'))

    patches_collection = PatchCollection(rectangles, match_original=True)
    ax.add_collection(patches_collection)

    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    plt.scatter(current.center[0], current.center[1])
    plt.scatter(all_points[:, 0], all_points[:, 1])

    # Add last reached
    if current.parent:
        plt.plot(current.parent.last_reached[0], current.parent.last_reached[1], '.k')
    plt.show()

def showFinal():
    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.add_patch(plt.Rectangle((-20, -20), 40, 40, fill=True, facecolor='white'))
    patches_collection = PatchCollection(rectangles, match_original=True)
    ax.add_collection(patches_collection)

    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    plt.scatter(all_points[:, 0], all_points[:, 1])

    plt.show()

def checkIntersection(x, y):
    r= drone_radius
    bb = quads.BoundingBox(min_x=x-r, max_x=x+r, min_y=y-r, max_y=y+r)
    intersections = tree.within_bb(bb)

    points = np.array([[i.x, i.y] for i in intersections])

    if len(points) == 0:
        return False

    fig, ax = plt.subplots()
    ax.add_patch(plt.Rectangle((x-r, y-r), 2*r, 2*r, fill=False))
    plt.xlim(-20, 20)
    plt.ylim(-4, 4)
    plt.scatter(points[:, 0], points[:, 1])
    plt.scatter(x, y)
    # plt.show()

    return True

if __name__ == "__main__":
    # Import points data
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

    # PARAMS
    drone_radius = 1
    map_size = 40 #Side Length of Map of Square shape
    depth = math.floor(math.log2(map_size/drone_radius))


    # Create a quadtree for querying points
    tree = quads.QuadTree((0, 0), map_size, map_size)

    for p in points:
        x = p[0]
        y = p[1]
        tree.insert((x, y))

    all_points = np.array(points)

    # Initialize sectors
    current_sector = Sector(-map_size/2, map_size/2, -map_size/2, map_size/2)
    init_point = [-15, -6]
    current_sector.path.append(init_point)
    current_sector.last_reached = init_point
    sector_queue = [current_sector]

    # Djikstra 
    planner = Dijkstra(all_points[:,0], all_points[:,1], 1.0, 1.0, 40, animate=False)
    pathCount = 0
    
    # Start from the first sector
    while len(sector_queue) > 0:
        checkSector(sector_queue.pop(0))

    showFinal()


    


    