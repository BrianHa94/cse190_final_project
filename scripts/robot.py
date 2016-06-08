#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import *
from cse_190_assi_3.msg import *
from read_config import read_config
from astar import *
from mdp import *

class Square():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.start = -1
        self.goal = -1
        self.wall = -1
        self.pit = -1
        self.MHDist = -1
        self.prev = 0
        self.steps = 0
        self.visited = -1
        self.reward = 0
        self.policy = 0
        self.shifted = 0
        self.hostage = 0

class Robot():
    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        rospy.sleep(2)
        self.saveCount = 0
        self.steps = 0

        # create necessary publishers
        self.MDPPub = rospy.Publisher('/results/policy_list', PolicyList, queue_size = 1000)
        self.completePub = rospy.Publisher('/map_node/sim_complete', Bool, queue_size = 1)
     
        # read in config file
        self.config = read_config()

        # construct map
        self.height = self.config['map_size'][0]
        self.width = self.config['map_size'][1]
        self.grid = []
        for i in range(0,self.height):
            self.grid.append([]) 
        for x in range(0,self.height):
            for y in range(0,self.width):
                self.grid[x].append(Square(x,y))

        # insert walls
        walls = self.config['walls']
        self.numOfWalls = len(walls)
        for i in range(0,len(walls)):
            self.grid[walls[i][0]][walls[i][1]].wall = 1
            self.grid[walls[i][0]][walls[i][1]].reward = "WALL"
            self.grid[walls[i][0]][walls[i][1]].policy = "WALL"

        # insert pits
        pits = self.config['pits']
        for i in range(0,len(pits)):
            self.grid[pits[i][0]][pits[i][1]].pit = 1
            self.grid[pits[i][0]][pits[i][1]].reward = "PIT"
            self.grid[pits[i][0]][pits[i][1]].policy = "PIT"
        
        # place hostages
        for i in range(0,len(self.config['goal'])):
            self.grid[self.config['goal'][i][0]][self.config['goal'][i][1]].hostage = 1

        # start and goal squares
        self.grid[self.config['start'][0]][self.config['start'][1]].start = 1
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].goal = 1
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].reward = "GOAL"
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].policy = "GOAL"
        self.start = self.config['start']
        self.goal = self.config['goal'][0]
  
        # run A* to get to first hostage to save (before walls begin to move)
        self.runAndPubAStar()
        
        # set start at prev goal & new goal at safe zone
        self.grid[self.config['start'][0]][self.config['start'][1]].start = -1
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].goal = -1
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].start = 1
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].reward = 0
        self.grid[self.config['goal'][0][0]][self.config['goal'][0][1]].policy = 0
        self.start = self.config['goal'][0]
        self.grid[self.start[0]][self.start[1]].hostage = 0

        self.goal = self.config['safe'][0]
        self.grid[self.config['safe'][0][0]][self.config['safe'][0][1]].goal = 1
        self.grid[self.config['safe'][0][0]][self.config['safe'][0][1]].reward = "GOAL"
        self.grid[self.config['safe'][0][0]][self.config['safe'][0][1]].policy = "GOAL"

        # publish warning
        warningGrid = []
        for x in range(0,self.height):
            for y in range(0,self.width):
                warningGrid.append("WARNING")                  
        PolicyMsg = PolicyList()
        PolicyMsg.data = warningGrid
        rospy.sleep(0.1)
        self.MDPPub.publish(PolicyMsg)
        self.pubGrid()
        PolicyMsg = PolicyList()
        PolicyMsg.data = warningGrid
        rospy.sleep(0.1)
        self.MDPPub.publish(PolicyMsg)
        self.pubGrid()

        # run MDP
        MDP(self.grid,self.height,self.width,self.config)

        # while loop, until all people are saved. Increment steps
        while self.saveCount != len(self.config['goal']) and self.steps < 1000:
            # attempt to move to next square according to MDP policy at current location
            nextX = 0
            nextY = 0
            if self.grid[self.start[0]][self.start[1]].policy == "N":
                nextX = -1
                nextY = 0
            elif self.grid[self.start[0]][self.start[1]].policy == "S":
                nextX = 1
                nextY = 0
            elif self.grid[self.start[0]][self.start[1]].policy == "W":
                nextX = 0
                nextY = -1
            elif self.grid[self.start[0]][self.start[1]].policy == "E":
                nextX = 0
                nextY = 1

            attemptX = self.start[0] + nextX
            attemptY = self.start[1] + nextY

            # attempted move is within map
            if attemptX >= 0 and attemptX < self.height and attemptY >= 0 and attemptY < self.width:
                # attempted move is not into wall or pit
                if self.grid[attemptX][attemptY].wall != 1 and self.grid[attemptX][attemptY].pit != 1:
                    self.grid[self.start[0]][self.start[1]].start = -1
                    self.grid[attemptX][attemptY].start = 1
                    self.start = [attemptX, attemptY]

            # publish move
            self.pubGrid()
            print self.start
            # if location is equal to goal and goal is safe spot
            if self.start[0] == self.goal[0] and self.goal[0] == self.config['safe'][self.saveCount][0] and self.start[1] == self.goal[1] and self.goal[1] == self.config['safe'][self.saveCount][1]:
                print "SAVED"
                self.saveCount += 1
                self.grid[self.start[0]][self.start[1]].hostage = 1
                self.unsetGoal()
                # still have more people to save
                if self.saveCount != len(self.config['goal']):
                    # set new goal to next person's location
                    self.goal = self.config['goal'][self.saveCount]
                    self.setGoal()
                    print "GETTING NEXT HOSTAGE"
                    # run MDP
                    MDP(self.grid,self.height,self.width,self.config)

                # else, all people have been saved. Head to original start location & Finish
                else:
                    # set goal to start location
                    self.goal = self.config['start']
                    self.setGoal()

                    # run A*
                    self.runAndPubAStar()
                    break

            # if location is equal to goal and goal is hostage spot
            if self.start[0] == self.goal[0] and self.goal[0] == self.config['goal'][self.saveCount][0] and self.start[1] == self.goal[1] and self.goal[1] == self.config['goal'][self.saveCount][1]:
                # reset goal to safe spot
                self.unsetGoal()
                self.grid[self.start[0]][self.start[1]].hostage = 0
                self.goal = self.config['safe'][self.saveCount]
                self.setGoal()

                # run MDP
                MDP(self.grid,self.height,self.width,self.config)

            self.steps += 1
            if self.steps % self.config['wallFreq'] == 0:
                # shift walls
                for x in range(0,self.height):
                    for y in range(0,self.width):
                        chance = random.randint(1,4)
                        if self.grid[x][y].wall == 1 and self.grid[x][y].shifted != 1 and chance == 1:
                            direction = random.randint(1,4)
                            wallX = 0
                            wallY = 0
                            # random north
                            if direction == 1:
                                wallX = -1
                                wallY = 0
                            # random south
                            elif direction == 2:
                                wallX = 1
                                wallY = 0
                            # random west
                            elif direction == 3:
                                wallX = 0
                                wallY = -1
                            # random east
                            elif direction == 4:
                                wallX = 0
                                wallY = 1

                            attemptWallX = x + wallX
                            attemptWallY = y + wallY

                            # attempted wall shift is within map and not in safe areas (first and last columns)
                            if attemptWallX >= 0 and attemptWallX < self.height and attemptWallY >= 1 and attemptWallY < self.width-1:
                                # attempted wall shift is not into wall or pit or robot
                                if self.grid[attemptWallX][attemptWallY].wall != 1 and self.grid[attemptWallX][attemptWallY].pit != 1 and self.grid[attemptWallX][attemptWallY].start != 1 and self.grid[attemptWallX][attemptWallY].goal != 1:
                                    self.grid[x][y].wall = -1
                                    self.grid[x][y].reward = 0
                                    self.grid[x][y].policy = 0
                                    self.grid[attemptWallX][attemptWallY].wall = 1
                                    self.grid[attemptWallX][attemptWallY].reward = "WALL"
                                    self.grid[attemptWallX][attemptWallY].policy = "WALL"
                                    self.grid[attemptWallX][attemptWallY].shifted = 1

                # reset shifted flag
                for x in range(0,self.height):
                    for y in range(0,self.width):
                        self.grid[x][y].shifted = 0

                # publish wall shift
                self.pubGrid()

                # run MDP
                MDP(self.grid,self.height,self.width,self.config)

            if self.steps % self.config['pitFreq'] == 0:
                # add pit
                pitX = random.randint(0, self.height - 1)
                pitY = random.randint(1, self.width - 2)
                randPitCount = 0
                while self.grid[pitX][pitY].start == 1 or self.grid[pitX][pitY].wall == 1 or self.grid[pitX][pitY].pit == 1 or self.grid[pitX][pitY].goal == 1:
                    pitX = random.randint(0, self.height - 1)
                    pitY = random.randint(1, self.width - 2)
                    randPitCount += 1
                    if randPitCount >= 8000:
                        break

                if randPitCount >= 8000:
                    break

                self.grid[pitX][pitY].pit = 1
                self.grid[pitX][pitY].reward = "PIT"
                self.grid[pitX][pitY].policy = "PIT"

                # publish new pit
                self.pubGrid()

                # run MDP
                MDP(self.grid,self.height,self.width,self.config)

        # done
        rospy.sleep(3)
        self.completePub.publish(Bool(True))
        rospy.sleep(3)
        rospy.signal_shutdown("Done...Signaling shutdown")


    def runAndPubAStar(self):
        # get Manhattan distances
        calcMHDist(self.grid,self.height,self.width,self.goal)

        # run A*
        pathList = AStar(self.grid,self.height,self.width,self.start,self.numOfWalls,self.config['move_list'])

        # iteratively move robot and publish every step
        for i in range(0,len(pathList)):
            mapGrid = []
            for x in range(0,self.height):
                for y in range(0,self.width):
                    if x == pathList[i][0] and y == pathList[i][1]:
                        mapGrid.append("R")
                    elif self.grid[x][y].hostage == 1:
                        mapGrid.append("H")
                    elif y == 0:
                        mapGrid.append("RED")
                    elif y == (self.width - 1):
                        mapGrid.append("GREEN")
                    elif self.grid[x][y].wall == 1:
                        mapGrid.append("WALL")
                    elif self.grid[x][y].pit == 1:
                        mapGrid.append("PIT")
                    #elif self.grid[x][y].goal == 1:
                    #    mapGrid.append("GOAL")
                    else:
                        mapGrid.append("NA")
            PolicyMsg = PolicyList()
            PolicyMsg.data = mapGrid
            rospy.sleep(0.1)
            self.MDPPub.publish(PolicyMsg)


    def pubGrid(self):
            mapGrid = []
            for x in range(0,self.height):
                for y in range(0,self.width):
                    if self.grid[x][y].start == 1:
                        mapGrid.append("R")
                    elif self.grid[x][y].hostage == 1:
                        mapGrid.append("H")
                    elif y == 0:
                        mapGrid.append("RED")
                    elif y == (self.width - 1):
                        mapGrid.append("GREEN")
                    elif self.grid[x][y].wall == 1:
                        mapGrid.append("WALL")
                    elif self.grid[x][y].pit == 1:
                        mapGrid.append("PIT")
                    #elif self.grid[x][y].goal == 1:
                    #    mapGrid.append("GOAL")
                    else:
                        mapGrid.append("NA")
            PolicyMsg = PolicyList()
            PolicyMsg.data = mapGrid
            rospy.sleep(0.1)
            self.MDPPub.publish(PolicyMsg)

    def unsetGoal(self):
        self.grid[self.goal[0]][self.goal[1]].goal = -1
        self.grid[self.goal[0]][self.goal[1]].reward = 0
        self.grid[self.goal[0]][self.goal[1]].policy = 0

    def setGoal(self):
        self.grid[self.goal[0]][self.goal[1]].goal = 1
        self.grid[self.goal[0]][self.goal[1]].reward = "GOAL"
        self.grid[self.goal[0]][self.goal[1]].policy = "GOAL"


if __name__ == '__main__':
    r = Robot()
