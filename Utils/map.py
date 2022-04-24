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
    map_len = 1000
    map_bre = 1000
    validPoints = set()
    d = radius + clearance
    # Defining Circle 1
    xc1 = 200
    yc1 = 200
    rc1 = 100 + d

    # Defining Circle 2
    xc2 = 200
    yc2 = 800
    rc2 = 100 + d

    # Defining Square and Rectangle 1
    x1 = 25 - d
    x2 = 175 + d
    x3 = 375 - d
    x4 = 625 + d
    y1 = 425 - d
    y2 = 575 + d

    # Defining Rectangle 2
    x5 = 725 - d
    x6 = 875 + d
    y6 = 400 + d
    y5 = 200 - d

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
