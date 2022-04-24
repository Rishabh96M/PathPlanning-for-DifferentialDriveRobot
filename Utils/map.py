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
    map_len = 100
    map_bre = 100
    validPoints = set()

    # Defining Circle 1
    xc1 = 20
    yc1 = 20
    rc1 = 10 + radius

    # Defining Circle 2
    xc2 = 20
    yc2 = 80
    rc2 = 10 + radius

    # Defining Square and Rectangle 1
    x1 = 2.5 - radius
    x2 = 17.5 + radius
    x3 = 37.5 - radius
    x4 = 62.5 + radius
    y1 = 42.5 - radius
    y2 = 57.5 + radius

    # Defining Rectangle 2
    x5 = 72.5 - radius
    x6 = 87.5 + radius
    y5 = 20.0 - radius
    y6 = 40.0 + radius

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
