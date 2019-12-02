#from mars_interface import *
import random
import intersect
import math
import numpy as np
 


class Node:

    def __init__(self, index, heuristic, f, parent=None):
        self.index = index
        self.heuristic = heuristic
        self.f = f
        self.parent = parent
    

    def set_parent(self, parent):
        self.parent = parent
    
    def set_f(self, f):
        self.f = f

    def get_h(self):
        return self.heuristic

    def get_index(self):
        return self.index
    
    def get_f(self):
        return self.f

    def get_G(self):
        return self.f-self.heuristic

    def __str__(self):
        return "index: "+ self.index + " heuristic: " + self.heuristic + " F: " + self.f + " parent: " + self.parent 


def checkIntersect(pos, wp):
    # for wall in walls:
    #     if intersect.doIntersect(line, wall):
    #         return False
    if intersect.get_intersect(pos[0], pos[1], wp[0], wp[1]) < 1:
       return False
    return True

def euclidean_distance(p1, p2):
    distance = np.sqrt(np.power(p1[0]-p2[0],2)+np.power(p1[1]-p2[1],2))
    return distance


def a_star(start_node, goal_node, list_nodes):
    # creating the open and closed lists
    open_list = []
    closed_list = []
    # Calculating the heuristic for the starting point
    h_start_goal = euclidean_distance(start_node, goal_node)
    # New node is created and it is inserted in open_list
    start = Node(np.where(list_nodes == start_node), h_start_goal, h_start_goal)
    open_list = np.append(open_list, start)
    # Until the open list is not empty...
    while open_list.size > 0:
        # Sort the open list in descending order of f = g + h
        open_list = sorted(open_list, key=lambda x: (x.get_f()), reverse=True)
        # Extract the minimum and remove from the open list
        min_node = open_list[0]
        open_list = np.delete(open_list,0)
        # Visiting the neighbors 
        for i in range(len(list_nodes)):
            # If a node is directly reachable from the current node, it is a neighbor of the current node
            if checkIntersect(min_node, list_nodes[i]):
                # If the neighbor is the goal_node add to the closed_list and end the search
                if list_nodes[i] == goal_node:
                    node = Node(list_nodes[i], 0, min_node.get_G() + euclidean_distance(list_nodes[min_node.get_index()], list_nodes[i]), min_node.get_index())
                    closed_list = np.append(closed_list, node)
                    break       
                else:
                    # Checking if the neighbor is already in the open_list
                    find = False
                    for j in range(len(closed_list)):
                        # if the neighbor is already in the closed_list and the value of F is higher than the current F...
                        current_F = min_node.get_G() + open_list[j].get_heuristic()
                        if closed_list[j].get_index() == i and closed_list[j].get_f() > current_F:
                            # Update the value of the F and the parent of the node
                            find = True
                            closed_list[j].set_f(current_F)
                            closed_list[j].set_parent(min_node.get_index())
                            break
                    # If the neighbor is not already in the closed_list, it will be pushed inside the open_list
                    if not find:
                        h = euclidean_distance(list_nodes[i],goal_node)
                        neighbor = Node(list_nodes[i], h, min_node.get_G() + euclidean_distance(list_nodes[min_node.get_index()], list_nodes[i]) + h, min_node.get_index())
                        open_list = np.append(open_list, neighbor)
            # the visited node is pushed inside the closed_list
            closed_list = np.append(closed_list, min_node)
    logMessage(closed_list)
    return closed_list
 
def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1, p2)))



def doBehavior(distance, marsData, waypoints, walls, pos):
    #print "doBehavior: " + str(pos[0])
    """
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)

    for wp in waypoints:
        if checkIntersect(pos,wp):
             #print str(pos[0]) + ":" + str(pos[1]) + " to " + str(wp[0]) + ":" + str(wp[1]) + ": no intersection"
             appendLines("path", pos[0], pos[1], 0.5)
             appendLines("path", wp[0], wp[1], 0.5)
    

            

#    message = "sensor:"
    """
    print(a_star(pos, [4,4], waypoints))
    return

pos = [0,0]
waypoints = [[5, 3.5], [4.2, -3.6], [-2, -3.6], [-6, -3.6],  [-4.5, 2.2], [-4, 4.8], [0, 4.5], [4.5, 0], [6, 0], \
    [-2.5, 0], [-1.2, 0], [10.2, 0], [4.5, 1.5], [6, -1.5], [-2.5, 1.5], [-1.2, -1.5], [0.2, 1.5], [-5, 0], [-5.7, 3], [-5.7, 4.5]]

print(a_star(pos, [4,4], waypoints))
