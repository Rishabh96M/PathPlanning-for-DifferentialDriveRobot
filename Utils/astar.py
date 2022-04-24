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


def getCost(curr_node, move, step, radius, w_dia):
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
        Th += (radius / w_dia) * (UR - UL) * dt
        D = D + math.sqrt(math.pow((0.5*radius*(UL + UR)
                                    * math.cos(Th)*dt), 2)+math.pow(
                                    (0.5*radius*(UL + UR)*math.sin(Th)*dt), 2))
    return (int(Xn), int(Yn), Th), D


def getAdjNodes(curr_node, validPoints, clearance, step, moves, radius, w_dia):
    """
    Definition
    ---
    Method to generate all adjacent nodes for a given node

    Parameters
    ---
    curr_node : node of intrest
    validPoints : list of all valid points
    clearance : minimum distance required from obstacles
    step : step size for each movement
    moves : action space of robot

    Returns
    ---
    adjNodes : list of adjacent nodes with cost from parent node
    """
    adjNodes = []
    for move in moves:
        # Checking if the point is valid
        new_node, cost = getCost(
            curr_node, move, step, radius, w_dia)
        x = new_node[0]
        y = new_node[1]
        if (x, y) in validPoints:
            adjNodes.append((new_node, cost))
    return adjNodes


def updateNode(new_node, curr_node, node_cost, queue, parent_map, cost, goal,
               thresh):
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

    Returns
    ---
    Reached : if new_node is goal node returns True othervise returns False
    node_cost : dict of all nodes mapped to costs
    queue : priority queue of nodes to check
    parent_map : dict of nodes mapped to parent node_cost
    """
    dist = abs(np.linalg.norm(np.asarray(
        new_node[0:2]) - np.asarray(goal[0:2])))
    new_cost = node_cost[curr_node] + cost + dist

    temp_cost = node_cost.get(new_node)
    if not temp_cost or (temp_cost > new_cost):
        node_cost[new_node] = new_cost
        parent_map[new_node[0:2]] = curr_node[0:2]
        heap.heappush(queue, (new_cost, new_node))
    if abs(np.linalg.norm(np.asarray(goal[0:2])
                          - np.asarray(new_node[0:2]))) < thresh:
        return True, node_cost, queue, parent_map
    return False, node_cost, queue, parent_map


def astar(start, goal, validPoints, clearance, step, thresh, rpm, radius,
          w_dia):
    """
    Definition
    ---
    Method to get least cost path from starting to goal node using dijkstra's

    Parameters
    ---
    start : starting node
    goal : goal node
    validPoints : list of all valid points
    clearance : minimum distance required from obstacles
    step : step size for each movement
    thresh : Threshold from goal point
    rpm: two possible wheel rpm's

    Returns
    ---
    Reached : if path is found True othervise False
    parent_map : dict of nodes mapped to parent node_cost
    closed : list of all the explored nodes
    """
    closed = []
    queue = []
    node_cost = {}
    parent_map = {}
    reached = False

    moves = [(rpm[0], 0), (0, rpm[0]), (rpm[0], rpm[0]), (rpm[1], 0),
             (0, rpm[1]), (rpm[1], rpm[0]), (rpm[0], rpm[1]), (rpm[1], rpm[1])]

    if abs(np.linalg.norm(np.asarray(goal[0:2])
                          - np.asarray(start[0:2]))) < thresh:
        reached = True
        parent_map[goal] = start[0:2]

    node_cost[start] = 0
    heap.heappush(queue, (0, start))
    while not reached and queue:
        curr_cost, curr_node = heap.heappop(queue)
        closed.append(curr_node[0:2])
        adjNodes = getAdjNodes(curr_node, validPoints, clearance, step, moves,
                               radius, w_dia)
        for new_node, cost in adjNodes:
            if new_node[0:2] in closed:
                continue
            flag, node_cost, queue, parent_map = updateNode(
                new_node, curr_node, node_cost, queue, parent_map, cost,
                goal, thresh)
            print('checking for node: ', new_node[0:2])
            if flag:
                closed.append(new_node[0:2])
                reached = True
                break
    return reached, parent_map, closed


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
    parent_node = parent_map[curr_node]
    path = [curr_node]
    while not parent_node == start[0:2]:
        curr_node = parent_node
        parent_node = parent_map[curr_node]
        path.append(curr_node)
    path.append(start[0:2])
    return path[::-1]


def animate(map_len, map_bre, validPoints, closed, path, parent_map):
    """
    Definition
    ---
    Method to animate the nodes explored by dijkstra's algorithm and plot the
    best path

    Parameters
    ---
    map_len : length of map
    map_bre : breadth of map
    validPoints : list of all valid points
    closed : list of all the explored nodes
    path: list of all the points from starting to goal position
    parent_map : dict of nodes mapped to parent node_cost
    """
    map_frame = np.zeros((map_bre + 1, map_len + 1, 3))
    resize = (400, 400)
    for point in validPoints:
        map_frame[map_bre - point[1], point[0]] = [255, 255, 255]
    cv2.circle(map_frame, (path[-1][0], map_bre
               - path[-1][1]), 2, [0, 0, 255], -1)
    cv2.circle(map_frame, (path[0][0], map_bre
               - path[0][1]), 2, [0, 255, 0], -1)
    for point in closed:
        if(point == path[0]):
            continue
        parent = parent_map[point]
        cv2.line(map_frame, (point[0], map_bre - point[1]),
                 (parent[0], map_bre - parent[1]), [255, 0, 0], 1)
        cv2.imshow('map_frame', cv2.resize(map_frame, resize))
        cv2.waitKey(1)
    for point in path:
        if(point == path[0]):
            continue
        parent = parent_map[point]
        cv2.line(map_frame, (point[0], map_bre - point[1]),
                 (parent[0], map_bre - parent[1]), [0, 255, 0], 1)
        cv2.imshow('map_frame', cv2.resize(map_frame, resize))
        cv2.waitKey(1)
    print('done, press any key to exit..')
    cv2.waitKey(0)
