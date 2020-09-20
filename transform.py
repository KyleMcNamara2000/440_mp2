
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
    dims = arm.getArmLimit()
    for i in range(len(dims)):
        dims[i] = math.floor((dims[i][1] - dims[i][0]) / granularity) + 1
    #print("dims:", dims)

    maze = []
    for i in range(dims[0]):
        column = []
        for j in range(dims[1]):
            column.append(" ")
        maze.append(column)
    #print(maze)
    '''
    maze[0][0] = "." #start point
    for i in range(dims[0]):
        for j in range(dims[1]):
            alpha = 100#idxToAngle(index, offsets, granularity)
            beta = 100#idxToAngle(index, offsets, granularity)
            arm.setArmAngle([alpha, beta])
            armPos = arm.getArmPos()
            if isArmWithinWindow(armPos, window) is False:
                maze[i][j] = "%"
            elif doesArmTouchObjects(armPos, obstacles, False) is True:
                maze[i][j] = "%" #can't go here
            elif doesArmTipTouchGoals(arm.getEnd(), goals) is True:
                maze[i][j] = "P" #solution
    '''
    pass