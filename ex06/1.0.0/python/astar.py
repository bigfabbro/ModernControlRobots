from mars_interface import *
import intersect
import math
import numpy as np

class Node:
    def __init__(self, pos, cost, parent):
        self.pos = pos
        self.cost = cost
        self.parent = parent

class CostList:
    def __init__(self):
        self.nodes = []

    def removeMin(self):
        minNode = self.nodes[0]
        for node in self.nodes:
            if node.cost < minNode.cost:
                minNode = node
        self.nodes.remove(minNode)
        return minNode
    
def checkLine(line, walls):
    # for wall in walls:
    #     if intersect.doIntersect(line, wall):
    #         return False
    if intersect.get_intersect(line[0], line[1], line[2], line[3]) < 1:
       return False
    return True

def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1, p2)))

def run(waypoints_, walls_, start, goal):
    path = None
    waypoints = np.copy(waypoints_)
    walls = np.copy(walls_)
    waypoints = np.append(waypoints, [start], 0)
    waypoints = np.append(waypoints, [goal], 0)
    
    startNode = Node(start, dist(start, goal), None)
    goalNode = Node(goal, 0, None)
    openList = CostList()
    openList.nodes.append(startNode)
    closedList = []

    while len(openList.nodes) > 0:
        currentNode = openList.removeMin()
        # finish
        if np.array_equal(currentNode.pos, goalNode.pos):
            while currentNode != startNode:
                if path is None:
                    path = np.array([currentNode.pos])
                else:
                    path = np.append(path, [currentNode.pos], 0)
                currentNode = currentNode.parent
            path = np.append(path, [startNode.pos], 0)
            return path

        # expand part
        for waypoint in waypoints:
            if np.array_equal(currentNode.pos, waypoint):
                continue
            lineToCheck = currentNode.pos
            lineToCheck = np.append(lineToCheck, waypoint)
            if not checkLine(lineToCheck, walls):
                continue
            found = False
            for node in closedList:
                if np.array_equal(node, waypoint):
                    found = True
                    break
            if found: continue

            cost = (currentNode.cost +
                    dist(currentNode.pos, waypoint) +
                    dist(waypoint, goal) -
                    dist(currentNode.pos, goal))
            addNode = True

            for node in openList.nodes:
                if np.array_equal(node.pos, waypoint):
                    if cost > node.cost:
                        addNode = False
                    else:
                        openList.nodes.remove(node)
                    break

            if not addNode: continue # handle next waypoint

            newNode = Node(waypoint, cost, currentNode)
            openList.nodes.append(newNode)
                                
        # add to closed list
        closedList.append(currentNode.pos)
    return path
