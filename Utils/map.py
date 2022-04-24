# map.py>
#
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: definig the given map


def listOfValidPoints(radius, clearance):
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
    d = radius + clearance
    # Defining Circle 1
    xc1 = 20
    yc1 = 20
    rc1 = 10 + d

    # Defining Circle 2
    xc2 = 20
    yc2 = 80
    rc2 = 10 + d

    # Defining Square and Rectangle 1
    x1 = 2.5 - d
    x2 = 17.5 + d
    x3 = 37.5 - d
    x4 = 62.5 + d
    y1 = 42.5 - d
    y2 = 57.5 + d

    # Defining Rectangle 2
    x5 = 72.5 - d
    x6 = 87.5 + d
    y6 = 40.0 + d
    y5 = 20.0 - d

    for x in range(int(d), map_len + 1 - int(d)):
        for y in range(int(d), map_bre + 1 - int(d)):
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
