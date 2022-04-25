# A Star path planning
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Using A star algorith to find the optimum path from staring
# to goal poisiton

import heapq as heap
import numpy as np
import math
import cv2


def getCost(curr_node, move, step, radius, w_dist, validPoints):
    """
    Definition
    ---
    Method to generate adjacent nodes for a given node and action along with
    the cost

    Parameters
    ---
    curr_node : node of intrest
    move : current action
    step : step size for each movement
    radius : radius of the robot
    w_dist : distance between the wheels of the robot
    validPoints : set of all valid points in the map

    Returns
    ---
    new_node : generated node based on action from current node
    D : cost to get to new node from current node
    """
    t = 0
    dt = 0.1
    Xn = curr_node[0]
    Yn = curr_node[1]
    UL = move[0]
    UR = move[1]
    Th = 3.14 * curr_node[2] / 180
    D = 0

    while t < step:
        t = t + dt
        Delta_Xn = 0.5 * radius * (UL + UR) * math.cos(Th) * dt
        Delta_Yn = 0.5 * radius * (UL + UR) * math.sin(Th) * dt
        Xn = Xn + Delta_Xn
        Yn = Yn + Delta_Yn
        if (round(Xn), round(Yn)) not in validPoints:
            return (-1, -1, -1), -1
        Th += (radius / w_dist) * (UR - UL) * dt
        D = D + math.sqrt(math.pow((0.5*radius*(UL + UR)
                                    * math.cos(Th)*dt), 2)+math.pow(
                                    (0.5*radius*(UL + UR)*math.sin(Th)*dt), 2))
    Th = 180 * Th / 3.14
    return (round(Xn), round(Yn), round(Th)), D


def getAdjNodes(curr_node, validPoints, step, moves, radius, w_dist):
    """
    Definition
    ---
    Method to generate all adjacent nodes for a given node

    Parameters
    ---
    curr_node : node of intrest
    validPoints : set of all valid points in the map
    step : step size for each movement
    moves : action space of robot
    radius : radius of the robot
    w_dist : distance between the wheels of the robot

    Returns
    ---
    adjNodes : list of adjacent nodes with cost from parent node
    """
    adjNodes = []
    for move in moves:
        # Checking if the point is valid
        new_node, cost = getCost(
            curr_node, move, step, radius, w_dist, validPoints)
        x = new_node[0]
        y = new_node[1]
        if (x, y) in validPoints:
            adjNodes.append((new_node, move, cost))
    return adjNodes


def updateNode(new_node, curr_node, node_cost, queue, parent_map, cost, goal,
               thresh, new_move):
    """
    Definition
    ---
    Method to update nodes based on cost and closed list of nodes

    Parameters
    ---
    new_node : node of intrest
    curr_node : parent node
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    cost : cost to get to new node from parent node
    goal : goal node
    thresh : Threshold from goal point
    new_move : action taken to reach new node from current node

    Returns
    ---
    Reached : if new_node is goal node returns True othervise returns False
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    """
    dist = math.sqrt(
        math.pow(new_node[0]-goal[0], 2) + math.pow(new_node[1]-goal[1], 2))
    new_cost = node_cost[curr_node] + cost + dist

    temp_cost = node_cost.get(new_node)
    if not temp_cost or (temp_cost > new_cost):
        node_cost[new_node] = new_cost
        parent_map[new_node] = (curr_node, new_move)
        heap.heappush(queue, (new_cost, new_node))
    if dist < thresh:
        return True, node_cost, queue, parent_map
    return False, node_cost, queue, parent_map


def astar(start, goal, validPoints, step, thresh, moves, radius, w_dist):
    """
    Definition
    ---
    Method to get least cost path from starting to goal node using dijkstra's

    Parameters
    ---
    start : starting node
    goal : goal node
    validPoints : list of all valid points in the map
    step : step size for each movement
    thresh : Threshold from goal point
    moves: action space of the robot
    radius : radius of the robot
    w_dist : distance between the wheels of the robot

    Returns
    ---
    Reached : if path is found True othervise False
    parent_map : dict of nodes mapped to parent node_cost
    closed : list of all the explored nodes
    """
    closed = []
    thresholded = set()
    queue = []
    node_cost = {}
    parent_map = {}
    reached = False

    if math.sqrt(math.pow(start[0]-goal[0], 2)
                 + math.pow(start[1]-goal[1], 2)) < thresh:
        reached = True
        parent_map[goal] = start[0:2]

    node_cost[start] = 0
    heap.heappush(queue, (0, start))
    while not reached and queue:
        curr_cost, curr_node = heap.heappop(queue)
        if visitedNode(thresholded, curr_node, thresh):
            continue
        closed.append(curr_node)
        thresholded.add((curr_node[0]//thresh, curr_node[1]//thresh))
        adjNodes = getAdjNodes(curr_node, validPoints, step, moves,
                               radius, w_dist)
        for new_node, new_move, cost in adjNodes:
            if visitedNode(thresholded, new_node, thresh):
                continue
            flag, node_cost, queue, parent_map = updateNode(
                new_node, curr_node, node_cost, queue, parent_map, cost,
                goal, thresh, new_move)
            print('checking for node: ', new_node[0:2])
            if flag:
                closed.append(new_node)
                thresholded.add((curr_node[0]//thresh, curr_node[1]//thresh))
                reached = True
                break
    return reached, parent_map, closed


def visitedNode(thresholded, curr_node, thresh):
    """
    Definition
    ---
    Method to check if node along with threshold has already been visited

    Parameters
    ---
    thresholded : set of all visited nodes after thresholding
    curr_node : node of interest
    thresh : threshold value

    Returns
    ---
    bool : True if alredy visited, False othervise
    """
    x = curr_node[0]//thresh
    y = curr_node[1]//thresh
    if (x, y) in thresholded:
        return True
    return False


def getPath(parent_map, start, goal, closed):
    """
    Definition
    ---
    Method to generate path using backtracking

    Parameters
    ---
    parent_map : dict of nodes mapped to parent node_cost
    start : starting node
    goal : goal node
    closed : list of all the explored nodes

    Returns
    ---
    path: list of all the points from starting to goal position
    """
    curr_node = closed[-1]
    path = [(curr_node, (0, 0))]
    while not curr_node[0:2] == start[0:2]:
        (parent_node, move) = parent_map[curr_node]
        path.append((parent_node, move))
        curr_node = parent_node
    return path[::-1]


def animate(map_len, map_bre, validPoints, closed, path, parent_map, moves,
            w_dist, radius, step):
    """
    Definition
    ---
    Method to animate the nodes explored by dijkstra's algorithm and plot the
    best path

    Parameters
    ---
    map_len : length of map
    map_bre : breadth of map
    validPoints : list of all valid points in the map
    closed : list of all the explored nodes
    path: list of all the points from starting to goal position
    parent_map : dict of nodes mapped to parent node_cost
    moves: action space of the robot
    radius : radius of the robot
    w_dist : distance between the wheels of the robot
    step : step size of each movement
    """
    map_frame = np.zeros((map_bre + 1, map_len + 1, 3))
    resize = (800, 800)
    for point in validPoints:
        map_frame[map_bre - point[1], point[0]] = [255, 255, 255]
    cv2.circle(map_frame, (path[-1][0][0], map_bre
               - path[-1][0][1]), 4, [0, 0, 255], -1)
    cv2.circle(map_frame, (path[0][0][0], map_bre
               - path[0][0][1]), 4, [0, 255, 0], -1)
    for point in closed:
        if point == path[-1][0]:
            continue
        for move in moves:
            Xs, Ys = curve_points(point, move, radius,
                                  w_dist, step, validPoints)
            for i in range(1, len(Xs)):
                cv2.line(map_frame, (Xs[i], map_bre - Ys[i]),
                         (Xs[i-1], map_bre - Ys[i-1]), [255, 0, 0], 2)
        cv2.imshow('map_frame', cv2.resize(map_frame, resize))
        cv2.waitKey(1)
    for point, move in path:
        Xs, Ys = curve_points(point, move, radius, w_dist, step, validPoints)
        for i in range(1, len(Xs)):
            cv2.line(map_frame, (Xs[i], map_bre - Ys[i]),
                     (Xs[i-1], map_bre - Ys[i-1]), [0, 0, 255], 4)
        cv2.imshow('map_frame', cv2.resize(map_frame, resize))
        cv2.waitKey(1)
    print('Path animation completed')
    print('press any key to to move turtlebot on ROS gazebo..')
    cv2.waitKey(0)


def curve_points(curr_node, move, radius, w_dist, step, validPoints):
    """
    Definition
    ---
    Method to generate points on a curve

    Parameters
    ---
    curr_node : node of interest
    move : current action
    radius : radius of the robot
    w_dist : distance between the wheels of the robot
    step : step size of each movement
    validPoints : set of all valid points in the map

    Returns
    ---
    Xs, Ys: list of all points on curve
    """
    t = 0
    dt = 0.1
    Xn = curr_node[0]
    Yn = curr_node[1]
    UL = move[0]
    UR = move[1]
    Thetan = 3.14 * curr_node[2] / 180
    Xs = []
    Ys = []
    while t < step:
        t = t + dt
        Xs.append(round(Xn))
        Ys.append(round(Yn))
        Xn += 0.5*radius * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*radius * (UL + UR) * math.sin(Thetan) * dt
        if (round(Xn), round(Yn)) not in validPoints:
            return [], []
        Thetan += (radius / w_dist) * (UR - UL) * dt
    return Xs, Ys
