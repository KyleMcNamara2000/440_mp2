
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
import math
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """

    #create new maze with dims = (range of angle) / granulity
    dims = arm.getArmLimit().copy()
    for i in range(len(dims)):
        dims[i] = int((dims[i][1] - dims[i][0]) / granularity) + 1
    #print("dims:", dims)

    maze = []
    for i in range(dims[0]):
        column = []
        for j in range(dims[1]):
            column.append(" ")
        maze.append(column)
    #print(maze)

    startAngles = []
    for i in range(len(arm.getArmAngle())):
        startAngles.append(int((arm.getArmAngle()[i] - arm.getArmLimit()[i][0]) / granularity))

    maze[startAngles[0]][startAngles[1]] = "P" #start point

    for i in range(dims[0]):
        for j in range(dims[1]):
            alpha = i * granularity + arm.getArmLimit()[0][0] #idxToAngle(index, offsets, granularity)
            beta = j * granularity + arm.getArmLimit()[1][0]
            arm.setArmAngle((alpha, beta))
            armPos = arm.getArmPos()

            if isArmWithinWindow(armPos, window) is False:
                maze[i][j] = "%"
            elif doesArmTouchObjects(arm.getArmPosDist(), obstacles, False) is True:
                maze[i][j] = "%"
            elif doesArmTipTouchGoals(arm.getEnd(), goals) is True:
                maze[i][j] = "."
            elif doesArmTouchObjects(arm.getArmPosDist(), goals, True) is True:
                maze[i][j] = "%"

            '''
            if isArmWithinWindow(armPos, window) is False:
                maze[i][j] = "%"
            elif doesArmTouchObjects(arm.getArmPosDist(), obstacles, False) is True:
                maze[i][j] = "%"
            else:
                flag = False
                for g in goals:
                    newG = goals.copy()
                    newG.remove(g)
                    if doesArmTipTouchGoals(arm.getEnd(), [g]) is True:
                        if doesArmTouchObjects(arm.getArmPosDist(), newG, True) is False:
                            maze[i][j] = "."
                            flag = True
                            break
                        else:
                            maze[i][j] = "%"
                            flag = True
                            break
                if flag is False:
                    if doesArmTouchObjects(arm.getArmPosDist(), goals, True) is True:
                        maze[i][j] = "%"
            '''


    retMaze = Maze(maze, (arm.getArmLimit()[0][0], arm.getArmLimit()[1][0]), granularity)
    retMaze.saveToFile("check2.txt")
    return retMaze