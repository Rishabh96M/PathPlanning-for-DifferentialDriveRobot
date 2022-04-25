# main.py
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Main method to calcualte optimal path using a-star

from Utils import map
from Utils import astar
from Utils import ros_talker

if __name__ == '__main__':
    map_len = 1000
    map_bre = 1000
    thresh = 10
    flag = True
    step = 1
    radius = 3.8
    w_dist = 35.4

    print('\n***********************************************************\n')
    print('Please not all inputs must in cm and degrees respectively')
    print('Map dimentions are 1000*1000')
    print('Ideal values for clearance are 0 to 10')
    print('Ideal values for rpm1, rpm2 are 10,20')
    print('\n***********************************************************\n')

    clearance = input('Input the clearance (ideal: 0-10):\n')
    clearance = int(clearance)
    if clearance < 0:
        flag = False
        print('Invalid input, please try again...')

    if flag:
        validPoints = map.listOfValidPoints(radius, clearance)

    if flag:
        start = input('Input Staring Position in format: x,y,th\n')
        start = (int(start.split(',')[0]), int(
            start.split(',')[1]), int(start.split(',')[2]))
        if start[0:2] not in validPoints:
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        goal = input('Input Goal Position in format: x,y\n')
        goal = (int(goal.split(',')[0]), int(
            goal.split(',')[1]))
        if goal not in validPoints:
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        rpm = input('Input two possible wheel RPM\'s in format: rpm1, rpm2\n')
        rpm = (int(rpm.split(',')[0]), int(rpm.split(',')[1]))
        moves = [(rpm[0], 0), (0, rpm[0]), (rpm[0], rpm[0]),
                 (rpm[1], 0), (0, rpm[1]), (rpm[1], rpm[0]),
                 (rpm[0], rpm[1]), (rpm[1], rpm[1])]

    if flag:
        print('starting')
        print(start)
        print(goal)
        reached, parent_map, closed = astar.astar(
            start, goal, validPoints, step, thresh, moves, radius,
            w_dist)
        if reached:
            print('\nreached')

            path = astar.getPath(parent_map, start, goal, closed)
            print('Path from goal to end is: ', path)

            astar.animate(map_len, map_bre, validPoints, closed, path,
                          parent_map, moves, w_dist, radius, step)

            ros_talker.send_vel(path, radius, w_dist, step)
            print('Simulation completed successfully')
        else:
            print('the point cannot be reached')
