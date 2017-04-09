#!/usr/bin/python3

# ==============================================================================
# Copyright 2017 Andrea Caltroni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
#
# v0.16 - 2017-02-07
# v0.1 - 2017-01-30
#
# Orders a matrix of integers, if possible, using the selected algorithm.
# python order-matrix.py [algorithm] [start_matrix]
# Usage:
# python order-matrix.py dfs 1,2,5,3,4,0,6,7,8
#
# Creates output file with:
# path_to_goal: ['Up', 'Left', 'Left']  | ['Up', 'Left', 'Left']
# cost_of_path: 3                       | 3
# nodes_expanded: 181437                | 181437
# fringe_size: 2                        | 2
# max_fringe_size: 42913                | 42913
# search_depth: 3                       | 3
# max_search_depth: 66125               | 66125
# running_time: 5.01608433              | 16.73173642
# max_ram_usage: 4.23940217             | 0
#
# What's New
# - fixed max_search_depth
# 
# TODO
# - max_ram_usage: not calculated yet
# - print to file
# - ast
# - ids
# - improve performance
#
# ==============================================================================

import math
#import resource
import sys
import time
from copy import deepcopy
from multiprocessing import Queue

# bfs (Breadth-First Search)
# dfs (Depth-First Search)
# ast (A-Star Search)
# ida (IDA-Star Search)
searchAlgs = ["bfs", "dfs", "ast", "ids"]

# directions the empty cell can move to
directions = ["U", "D", "L", "R"]
directionsReversed = ["R", "L", "D", "U"]


# ==============================================================================
# Classes
# ==============================================================================

# ------------------------------------------------------------------------------
# Statistics variables
# ------------------------------------------------------------------------------
class Statistics:
    def __init__(self):
        self.pathToGoal = []
        self.nodesExpanded = 0
        self.fringeSize = 0
        self.maxFringeSize = 0
        self.searchDepth = 0
        self.maxSearchDepth = 0
        self.maxRamUsage = 0
        self.startTime = 0
        self.endTime = 0
    
    def getPathToGoal(self):
        return self.pathToGoal

    def setPathToGoal(self, pathToGoal):
        self.pathToGoal = pathToGoal

    def getNodesExpanded(self):
        return self.nodesExpanded

    def increaseNodesExpanded(self):
        self.nodesExpanded = self.nodesExpanded + 1

    def getFringeSize(self):
        return self.fringeSize

    def setFringeSize(self, fringeSize):
        self.fringeSize = fringeSize

    def getMaxFringeSize(self):
        return self.maxFringeSize

    def setMaxFringeSize(self, maxFringeSize):
        self.maxFringeSize = maxFringeSize

    def getSearchDepth(self):
        return self.searchDepth

    def setSearchDepth(self, searchDepth):
        self.searchDepth = searchDepth

    def getMaxSearchDepth(self):
        return self.maxSearchDepth

    def setMaxSearchDepth(self, maxSearchDepth):
        self.maxSearchDepth = maxSearchDepth

    def getStartTime(self):
        return self.startTime
        
    def setStartTime(self, startTime):
        self.startTime = startTime

    def setEndTime(self, endTime):
        self.endTime = endTime

    def getMaxRamUsage(self):
        return self.maxRamUsage

    def setMaxRamUsage(self, maxRamUsage):
        self.maxRamUsage = maxRamUsage

    # TODO print to file instead
    def printStats(self):
        print("")
        print("path_to_goal: ", self.pathToGoal)
        print("cost_of_path: ", len(self.pathToGoal))
        print("nodes_expanded: ", self.nodesExpanded)
        print("fringe_size: ", self.fringeSize)
        print("max_fringe_size: ", self.maxFringeSize)
        print("search_depth: ", self.searchDepth)
        print("max_search_depth: ", self.maxSearchDepth)
        print("running_time: ", round(self.endTime - self.startTime, 8))
        print("max_ram_usage: ", self.maxRamUsage)


# ------------------------------------------------------------------------------
# A tiles in the exploration process
# ------------------------------------------------------------------------------
class Board:
    def __init__(self):
        self.tiles = []
        self.emptyCell = []
        # the list of moves to reach this configuration
        self.pathToGoal = []
        self.searchDepth = 0
        self.parent = None
        self.direction = None

    def getTiles(self):
        return self.tiles

    def setTiles(self, tiles):
        self.tiles = tiles

    def getEmptyCell(self):
        return self.emptyCell
        
    def setEmptyCell(self, emptyCell):
        self.emptyCell = emptyCell

    def getSearchDepth(self):
        return self.searchDepth

    def setSearchDepth(self, searchDepth):
        self.searchDepth = searchDepth

    def getParent(self):
        return self.parent

    def setParent(self, parent):
        self.parent = parent

    def getDirection(self):
        return self.direction

    def setDirection(self, direction):
        self.direction = direction

    # Converts a tiles into a string (with a trailing comma, not relevant)
    @staticmethod
    def toString(tiles):
        outStr = ""
        tilesSize = len(tiles)
        for i in range(tilesSize):
            for j in range(tilesSize):
                outStr = outStr + str(tiles[i][j]) + ","
        return outStr[0:-1]


# ==============================================================================
# Functions
# ==============================================================================

# ------------------------------------------------------------------------------
# Moves the empty cell in the given direction if possible
# ------------------------------------------------------------------------------
def move(board, direction):
    emptyCell = board.getEmptyCell()
    row = emptyCell[0]
    col = emptyCell[1]
    tiles = board.getTiles()
    tilesSize = len(tiles)
    if (direction == "U"):
        # Up
        if (row > 0):
            tiles[row][col] = tiles[row-1][col]
            tiles[row-1][col] = 0
            emptyCell[0] = emptyCell[0] - 1
            return True
        else:
            return False
    elif (direction == "D"):
        # Down
        if (row < tilesSize - 1):
            tiles[row][col] = tiles[row+1][col]
            tiles[row+1][col] = 0
            emptyCell[0] = emptyCell[0] + 1
            return True
        else:
            return False
    elif (direction == "L"):
        # Left
        if (col > 0):
            tiles[row][col] = tiles[row][col-1]
            tiles[row][col-1] = 0
            emptyCell[1] = emptyCell[1] - 1
            return True
        else:
            return False
    elif (direction == "R"):
        # Right
        if (col < tilesSize - 1):
            tiles[row][col] = tiles[row][col+1]
            tiles[row][col+1] = 0
            emptyCell[1] = emptyCell[1] + 1
            return True
        else:
            return False


def traverse(board):
    path = []
    tmpBoard = board
    while (tmpBoard.getParent()):
        direction = tmpBoard.getDirection()
        if (direction == "U"):
            path.append("Up")
        elif (direction == "D"):
            path.append("Down")
        elif (direction == "L"):
            path.append("Left")
        elif (direction == "R"):
            path.append("Right")
        tmpBoard = tmpBoard.getParent()
    path.reverse()
    return path


# ------------------------------------------------------------------------------
# Breadth First Search (BFS)
# ------------------------------------------------------------------------------
def bfs(tiles, startEmptyCell, successTiles):
    print("Executing bfs")
    stats = Statistics()
    stats.setStartTime(time.time())
    # FIFO queue for the matrixes to be examined
    fringe = Queue()
    # Used to check if a tiles needs to be examined or not
    fringeOfStrings = set()
    # add initial tiles to the fringe and checked set
    initialBoard = Board()
    initialBoard.setTiles(tiles)
    initialBoard.setEmptyCell(startEmptyCell)
    # parent is already set
    fringe.put(initialBoard)
    fringeOfStrings.add(Board.toString(tiles))
    stats.setMaxFringeSize(1)
    maxRamUsage = 0

    success = False

    while (not fringe.empty()):
        nextBoard = fringe.get()
        nextTiles = nextBoard.getTiles()
        
        if (nextTiles == successTiles):
            stats.setPathToGoal(traverse(nextBoard))
            stats.setSearchDepth(nextBoard.getSearchDepth())
            print("")
            print("Success!")
            success = True
            break
        else:
            stats.increaseNodesExpanded()
            # add children to fringe (FIFO)
            # visit child nodes in the "UDLR" order
            nextEmptyCell = nextBoard.getEmptyCell()
            for direction in directions:
                tmpChildBoard = Board()
                tmpChildBoard.setTiles(deepcopy(nextTiles))
                tmpChildBoard.setEmptyCell(deepcopy(nextEmptyCell))
                # changes the tiles and the next empty cell
                moveResult = move(tmpChildBoard, direction)
                if (moveResult == True):
                    tmpStrTiles = Board.toString(tmpChildBoard.getTiles())
                    if (tmpStrTiles not in fringeOfStrings):
                        tmpChildBoard.setDirection(direction)
                        fringe.put(tmpChildBoard)
                        fringeOfStrings.add(tmpStrTiles)
                        if (fringe.qsize() > stats.getMaxFringeSize()):
                            stats.setMaxFringeSize(fringe.qsize())
                        tmpSearchDepth = nextBoard.getSearchDepth() + 1
                        tmpChildBoard.setSearchDepth(tmpSearchDepth)
                        if (tmpSearchDepth > stats.getMaxSearchDepth()):
                            stats.setMaxSearchDepth(tmpSearchDepth)
                        tmpChildBoard.setParent(nextBoard)

            # max RAM usage
            #ramUsage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            #if (ramUsage > maxRamUsage):
            #    maxRamUsage = ramUsage
                
    if (not success):
        print("")
        print("Solution not found (empty fringe)")

    stats.setFringeSize(fringe.qsize())
    stats.setEndTime(time.time())
    stats.setMaxRamUsage(maxRamUsage / 1000)
    return stats


# ------------------------------------------------------------------------------
# Depth First Search (DFS)
# path_to_goal: ['Up', 'Left', 'Left']

# python driver.py dfs 1,2,5,3,4,0,6,7,8
# cost_of_path: 3
# nodes_expanded: 181437
# fringe_size: 2
# max_fringe_size: 42913
# search_depth: 3
# max_search_depth: 66125
# ------------------------------------------------------------------------------
def dfs(tiles, startEmptyCell, successTiles):
    print("Executing dfs")
    stats = Statistics()
    stats.setStartTime(time.time())
    # LIFO queue for the matrixes to be examined
    # use append() to add, pop() with no index to retrieve from the top
    fringe = []
    # Used to check if a tiles needs to be examined or not
    fringeOfStrings = set()
    # add initial tiles to the fringe and checked set
    initialBoard = Board()
    initialBoard.setTiles(tiles)
    initialBoard.setEmptyCell(startEmptyCell)
    # parent is already set
    fringe.append(initialBoard)
    fringeOfStrings.add(Board.toString(tiles))
    stats.setMaxFringeSize(1)
    maxRamUsage = 0

    success = False

    while (len(fringe) > 0):
        nextBoard = fringe.pop()
        nextTiles = nextBoard.getTiles()
        
        if (nextTiles == successTiles):
            stats.setPathToGoal(traverse(nextBoard))
            stats.setSearchDepth(nextBoard.getSearchDepth())
            print("")
            print("Success!")
            success = True
            break
        else:
            stats.increaseNodesExpanded()
            # add children to fringe (LIFO).
            # visit child nodes in the "RLDU" order to have the last one
            # on top of the stack.
            nextEmptyCell = nextBoard.getEmptyCell()
            for direction in directionsReversed:
                tmpChildBoard = Board()
                tmpChildBoard.setTiles(deepcopy(nextTiles))
                tmpChildBoard.setEmptyCell(deepcopy(nextEmptyCell))
                moveResult = move(tmpChildBoard, direction)
                if (moveResult == True):
                    tmpStrTiles = Board.toString(tmpChildBoard.getTiles())
                    if (tmpStrTiles not in fringeOfStrings):
                        tmpChildBoard.setDirection(direction)
                        fringe.append(tmpChildBoard)
                        if (len(fringe) > stats.getMaxFringeSize()):
                            stats.setMaxFringeSize(len(fringe))
                        fringeOfStrings.add(tmpStrTiles)
                        tmpSearchDepth = nextBoard.getSearchDepth() + 1
                        tmpChildBoard.setSearchDepth(tmpSearchDepth)
                        if (tmpSearchDepth > stats.getMaxSearchDepth()):
                            stats.setMaxSearchDepth(tmpSearchDepth)
                        tmpChildBoard.setParent(nextBoard)
            # max RAM usage
            #ramUsage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            #if (ramUsage > maxRamUsage):
            #    maxRamUsage = ramUsage
                
    if (not success):
        print("")
        print("Solution not found (empty fringe)")

    stats.setFringeSize(len(fringe))
    stats.setEndTime(time.time())
    stats.setMaxRamUsage(maxRamUsage / 1000)
    return stats


# ==============================================================================
# Main
# ==============================================================================

searchAlg = ""
if (len(sys.argv) == 3):
    # read search alg
    if ((sys.argv[1] in searchAlgs)):
        searchAlg = sys.argv[1]
    else:
        sys.exit("Search algorithm argument invalid! Terminating.")

    # read tiles (no check for a valid input)
    tilesArray = sys.argv[2].split(',')
else:
    sys.exit("Wrong number of arguments! Terminating.")

# --- initial tiles
tilesSize = int(math.sqrt(len(tilesArray)))

arrayIdx = 0
# range goes to param - 1
tiles = [[0 for x in range(tilesSize)] for y in range(tilesSize)]
for i in range(tilesSize):
    for j in range(tilesSize):
        tiles[i][j] = int(tilesArray[arrayIdx])
        arrayIdx = arrayIdx + 1

# empty cell of the initial tiles
startEmptyCell = [-1, -1]
# find the 0
for i in range(tilesSize):
    for j in range(tilesSize):
        if (tiles[i][j] == 0):
            startEmptyCell[0] = i
            startEmptyCell[1] = j
            break

print("Initial tiles: ", tiles)

# --- success tiles
successTiles = [[0 for x in range(tilesSize)] for y in range(tilesSize)]

cellValue = 0;
for i in range(tilesSize):
    for j in range(tilesSize):
        successTiles[i][j] = cellValue
        cellValue = cellValue + 1

print("Success tiles: ", successTiles)


if (searchAlg == "bfs"):
    stats = bfs(tiles, startEmptyCell, successTiles)
elif (searchAlg == "dfs"):
    stats = dfs(tiles, startEmptyCell, successTiles)
else:
    sys.exit("*** Not implemented yet!")

stats.printStats()
