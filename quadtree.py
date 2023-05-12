from pyqtree import Index
import quads
import numpy as np
import json
import math
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.collections import PatchCollection

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
    if obstacles == 0: # and current.parent.reachable == True:
        current.color = 'green'
        current.reachable = True # set current.reachable = True when its both empty and a path exists
        animate(current.color, current)
        return
    
    #check reachability from last point in path to the sector's center
    # n = len(current.path)
    # while n:
    #     # if checkPath(current.center, current.path[n-1]):
    #     if True:
    #         # Found a path
    #         current.reachable = True
    #         break
    #     else:
    #         # Check for previous point in the path
    #         n = n - 1

    # sector = current
    # while sector:
    #     if True: # check if path exists, but ignoring the check right now, assume every sector is reachable
    #         current.last_reached = current.center
    #         break
    #     else:
    #         sector = current.parent

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
        left_top = Sector(xmin, xmin+child_size, ymin+child_size, ymax)
        left_top.parent = current
        right_bottom = Sector(xmin+child_size, xmax, ymin, ymin+child_size)
        right_bottom.parent = current
        right_top = Sector(xmin+child_size, xmax, ymin+child_size, ymax)
        right_top.parent = current

        sector_queue.append(left_bottom)
        sector_queue.append(left_top)
        sector_queue.append(right_bottom)
        sector_queue.append(right_top)

    animate(current.color, current)

rectangles = []

def animate(c, current):
    print(c)
    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.add_patch(plt.Rectangle((-20, -20), 40, 40, fill=True, facecolor='white'))
    
    # if current.parent:
    #     rectangles.append(plt.Rectangle((current.parent.center[0]-current.parent.size/2, current.parent.center[1]-current.parent.size/2), current.parent.size, current.parent.size
    #                             , fill=True, facecolor=current.parent.color, edgecolor='black'))
    rectangles.append(plt.Rectangle((current.center[0]-current.size/2, current.center[1]-current.size/2), current.size, current.size, fill=True, facecolor=c, edgecolor='black'))

    # if current.parent:
    #     ax.add_patch(plt.Rectangle((current.parent.center[0]-current.parent.size/2, current.parent.center[1]-current.parent.size/2), current.parent.size, current.parent.size
    #                             , fill=True, facecolor=current.parent.color, edgecolor='black'))
    # ax.add_patch(plt.Rectangle((current.center[0]-current.size/2, current.center[1]-current.size/2), current.size, current.size, fill=True, facecolor=c, edgecolor='black'))

    for r in rectangles:
        print(r)

    patches_collection = PatchCollection(rectangles, match_original=True)
    ax.add_collection(patches_collection)

    plt.xlim(-30, 30)
    plt.ylim(-30, 30)
    plt.scatter(current.center[0], current.center[1])
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

    drone_radius = 1
    map_size = 40 #Side Length of Map of Square shape
    depth = math.floor(math.log2(map_size/drone_radius))

    # spindex = Index(bbox=(-20, 20, -20, 20), max_items=len(points), max_depth=depth)
    tree = quads.QuadTree((0, 0), map_size, map_size)

    for p in points:
        x = p[0]
        y = p[1]
        tree.insert((x, y))

    all_points = np.array(points)

    current_sector = Sector(-map_size/2, map_size/2, -map_size/2, map_size/2)

    init_point = [-15, -6]
    current_sector.path.append(init_point)
    sector_queue = [current_sector]
    
    while len(sector_queue) > 0:
        checkSector(sector_queue.pop(0))


    


    