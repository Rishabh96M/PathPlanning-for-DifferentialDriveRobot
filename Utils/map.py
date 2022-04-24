# map.py>
#
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: definig the given map


def listOfValidPoints(radius):
    """
    Definition
    ---
    Method to generate a list of all valid points on a map

    Returns
    ---
    validPoints : set of all the valid points
    """
    map_len = 1000
    map_bre = 1000
    validPoints = set()

    # Defining Circle 1
    xc1 = 200
    yc1 = 200
    rc1 = 100 + radius

    # Defining Circle 2
    xc2 = 200
    yc2 = 800
    rc2 = 100 + radius

    # Defining Square and Rectangle 1
    x1 = 25 - radius
    x2 = 175 + radius
    x3 = 375 - radius
    x4 = 625 + radius
    y1 = 425 - radius
    y2 = 575 + radius

    # Defining Rectangle 2
    x5 = 725 - radius
    x6 = 875 + radius
    y5 = 200 - radius
    y6 = 400 + radius

    for x in range(int(radius), map_len + 1 - int(radius)):
        for y in range(int(radius), map_bre + 1 - int(radius)):
            if ((x - xc1)**2 + (y - yc1)**2) <= rc1**2:
                continue
            if ((x - xc2)**2 + (y - yc2)**2) <= rc2**2:
                continue
            if y > y1 and y < y2:
                if x > x1 and x < x2:
                    continue
                if x > x3 and x < x4:
                    continue
            if y > y5 and y < y6 and x > x5 and x < x6:
                continue
            validPoints.add((x, y))
    return validPoints


def isPointValid(point, validPoints, clearance):
    """
    Definition
    ---
    Method to check if point is valid and clear of obstacles

    Parameters
    ---
    point : node of intrest
    validPoints : set of all valid points
    clearance : minimum distance required from obstacles

    Returns
    ---
    bool : True if point is valid, False othervise
    """
    for i in range(-clearance, clearance):
        for j in range(-clearance, clearance):
            if not (point[0] + i, point[1] + j) in validPoints:
                return False
    return True
