import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import json
from random import random

from rrt_star import RRTStar

DRONE_SIZE = 1

def findNeighours(p, r_):
    ind = tree.query_radius([p], r=r_)
    
    neighbours = []
    for i in ind[0]:
        neighbours.append(points[i])

    neighbours = np.array(neighbours)
    if neighbours.ndim == 1:
        return False
    
    fig, ax = plt.subplots()
    plt.xlim(-20, 10)
    plt.ylim(-4, 4)
    plt.scatter(neighbours[:,0], neighbours[:,1])
    plt.scatter(p[0], p[1])
    ax.add_patch(plt.Circle((p[0], p[1]), r_, fill=False))
    plt.show()

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

    # plt.scatter(x_points, y_points)
    # plt.show()
    tree = KDTree(points)

    ###################

    prev_centers = [] # List holding the last sector center to which an RRT path was found

    sector = [-20, 20, -20, 20]

    x_len = sector[1] - sector[0]
    y_len = sector[3] - sector[2]

    center = [x_len/2, y_len/2]

    # CHECK RRT TO CENTER

    prev_centers.append(center)

    subsector1 = [sector[0], sector[0]+x_len/2, sector[2], sector[2]+y_len/2] # bottom left
    subsector2 = [sector[0]+x_len/2, sector[1], sector[2], sector[2]+y_len/2] # bottom right
    subsector3 = [sector[0], sector[0]+x_len/2, sector[2]+y_len/2, sector[3]] # top left
    subsector4 = [sector[0]+x_len/2, sector[1], sector[2]+y_len/2, sector[3]] # top right
    subsectors = [subsector1, subsector3,subsector3,subsector4]

    # CHECK NEIGHBOURS IN SUBSECTOR
    subsectors_to_check = []
    for s in subsectors:
        print(s)
        radius = (s[1] - s[0])/2
        center_x = s[0] + radius
        center_y = s[2] + radius
        # print(radius, center_x, center_y)
        findNeighours([center_x, center_y], radius)
        if findNeighours:   
            subsectors_to_check.append(s)

    # Repeat for every subsector

    # rrt_star = RRTStar(
    #     start=[-15, -3],
    #     goal=[5, 3],
    #     rand_area=[-5, 5],
    #     obstacle_list=tree,
    #     points=points,
    #     expand_dis=1,
    #     robot_radius=1,
    #     max_iter=500)
    
    # path = rrt_star.planning(animation=False)

    # rrt_star.draw_graph()
    # plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    # plt.grid(True)
    # plt.pause(0.01)  # Need for Mac
    # plt.show()
