# main.py
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Main method to calcualte optimal path using a-star

from Utils import map
from Utils import astar

if __name__ == '__main__':
    map_len = 10
    map_bre = 10
    thresh = 0.1
    flag = True
    step = 1
    radius = 0.105
    w_dia = 0.066
    clearance = 0
    validPoints = set()

    print('Please not all inputs must in meters and degrees respectively\n\n')
    clearance = input('Input the clearance in m (0 - 0.25m):\n')
    clearance = int(clearance)
    if clearance < 0:
        flag = False
        print('Invalid input, please try again...')

    if flag:
        validPoints = map.listOfValidPoints(map_len, map_bre, radius)

    if flag:
        start = input('Input Staring Position in format: x,y,th\n')
        start = (int(start.split(',')[0]), int(
            start.split(',')[1]), int(start.split(',')[2]))
        if not map.isPointValid(start[0:2], validPoints, clearance):
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        goal = input('Input Staring Position in format: x,y\n')
        goal = (int(goal.split(',')[0]), int(
            goal.split(',')[1]))
        if not map.isPointValid(goal, validPoints, clearance):
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        rpm = input('Input two possible wheel RPM\'s in format: rpm1, rpm2')
        rpm = (int(rpm.split(',')[0]), int(rpm.split(',')[1]))

    if flag:
        print('starting')
        print(start)
        print(goal)
        reached, parent_map, closed, pointsToPlot = astar.astar(
            start, goal, validPoints, clearance, step, thresh, rpm)
        if reached:
            print('reached')
            path = astar.getPath(parent_map, start, goal, closed, pointsToPlot)
            print(path)
            astar.animate(map_len, map_bre, validPoints,
                          closed, path, parent_map)
        else:
            print('the point cannot be reached')
