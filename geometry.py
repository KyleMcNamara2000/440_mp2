# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains geometry functions that relate with Part1 in MP2.
"""

import math
import numpy as np
from const import *



def computeCoordinate(start, length, angle):

    d_x = int(length * np.cos(np.radians(angle)))
    d_y = int(length * np.sin(np.radians(angle)))
    newPoint = (start[0] + d_x, start[1] - d_y)
    return newPoint


#copied from https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def dist(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
    px = x2-x1
    py = y2-y1

    norm = px*px + py*py

    u =  float((x3 - x1) * px + (y3 - y1) * py) / float(norm)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    # Note: If the actual distance does not matter,
    # if you only want to compare what this function
    # returns to other results of this function, you
    # can just return the squared distance instead
    # (i.e. remove the sqrt) to gain a little performance

    dist = (float(dx*dx + dy*dy))**.5

    return dist


#line: (m, b), point: (x, y), return con point (x, y)
def findConnectPoint(line, point):
    if line is None:
        return None
    if line[0] == 0:
        return (point[0], line[1])
    perpSlope = -1.0 / line[0]
    line2 = getMandBSlope(point, perpSlope)
    m = line[0]
    b1 = line[1]
    b2 = line2[1]
    #intersection of 2 perp lines = (m(b2-b1)/(m^2+1), (m^2*b2 + b1) / (m^2 + 1))
    return (m * (b2 - b1) / (m**2 + 1.0), (m**2 * b2 + b1) / (m**2 + 1.0))


#return (m, b) from points good
def getMandBPoints(point1, point2):
    if point2[0] - point1[0] == 0:
        return None
    return getMandBSlope(point1, ((float)(point2[1] - point1[1])) / ((float)(point2[0] - point1[0])))

# return (m, b) from point and slope good
def getMandBSlope(point, slope):
    #b = y1 - x1(m)
    return (slope, point[1] - point[0] * slope)

#good
def euclidDist(p1, p2):
    return math.sqrt(float((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2))


def doesArmTouchObjects(armPosDist, objects, isGoal=False):
    #for segment in segs:
    for link in armPosDist:
        #for object in objs:
        for object in objects:
            '''
            #calc point of intersection on line
            mb = getMandBPoints(link[0], link[1])
            conPoint = findConnectPoint(mb, (object[0], object[1]))
            distance = 0
            flag = False
            if conPoint is None:
                conPoint = (link[0][0], object[1])
            if conPoint[0] <= max(link[0][0], link[1][0]) and conPoint[0] >= min(link[0][0], link[1][0]) and conPoint[1] <= max(link[0][1], link[1][1]) and conPoint[1] >= min(link[0][1], link[1][1]):
                # if intersect in segment -> return that distance
                distance = euclidDist(conPoint, object)
                flag = True
            else:
                #else -> return min(distance to each endpoint)
                distance = min(euclidDist(link[0], object), euclidDist(link[1], object))
            '''

            distance = dist(link[0][0], link[0][1], link[1][0], link[1][1], object[0], object[1])

            buffer = link[2]
            if isGoal:
                buffer = 0
            if distance <= object[2] + buffer:
                #print("object:", object, "making contact with", link, "in seg?", flag)
                return True


    return False




def doesArmTipTouchGoals(armEnd, goals):
    """Determine whether the given arm tip touch goals

        Args:
            armEnd (tuple): the arm tick position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]. There can be more than one goal.
        Return:
            True if arm tip touches any goal. False if not.
    """
    for g in goals:
        if np.linalg.norm((g[0] - armEnd[0], g[1] - armEnd[1])) <= g[2]:
            return True
    return False


def isArmWithinWindow(armPos, window):
    """Determine whether the given arm stays in the window

        Args:
            armPos (list): start and end positions of all arm links [(start, end)]
            window (tuple): (width, height) of the window

        Return:
            True if all parts are in the window. False if not.
    """
    #check if all 3 points are in window, if so we good
    for joint in armPos:
        for p in joint:
            if p in armPos[0][0]:
                continue
            if p[0] < 0 or p[0] > window[0] or p[1] < 0 or p[1] > window[1]:
                return False
    return True


if __name__ == '__main__':
    computeCoordinateParameters = [((150, 190),100,20), ((150, 190),100,40), ((150, 190),100,60), ((150, 190),100,160)]
    resultComputeCoordinate = [(243, 156), (226, 126), (200, 104), (57, 156)]
    testRestuls = [computeCoordinate(start, length, angle) for start, length, angle in computeCoordinateParameters]

    assert testRestuls == resultComputeCoordinate

    testArmPosDists = [((100,100), (135, 110), 4), ((135, 110), (150, 150), 5)]
    testObstacles = [[(120, 100, 5)], [(110, 110, 20)], [(160, 160, 5)], [(130, 105, 10)]]
    resultDoesArmTouchObjects = [
        True, True, False, True, False, True, False, True,
        False, True, False, True, False, False, False, True
    ]

    testResults = []
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle))

    print("\n")
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))
    #print(resultDoesArmTouchObjects, "\n", testResults)
    assert resultDoesArmTouchObjects == testResults

    testArmEnds = [(100, 100), (95, 95), (90, 90)]
    testGoal = [(100, 100, 10)]
    resultDoesArmTouchGoals = [True, True, False]

    testResults = [doesArmTipTouchGoals(testArmEnd, testGoal) for testArmEnd in testArmEnds]
    assert resultDoesArmTouchGoals == testResults

    testArmPoss = [((100,100), (135, 110)), ((135, 110), (150, 150))]
    testWindows = [(160, 130), (130, 170), (200, 200)]
    resultIsArmWithinWindow = [True, False, True, False, False, True]
    testResults = []
    for testArmPos in testArmPoss:
        for testWindow in testWindows:
            testResults.append(isArmWithinWindow([testArmPos], testWindow))
    assert resultIsArmWithinWindow == testResults

    print("Test passed\n")
