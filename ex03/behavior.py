from mars_interface import *
import random
import intersect
import heapq
import math
import numpy as np
 


class Node:

    def __init__(self, index, heuristic, f, parent = None):
        self.index = index
        self.heuristic = heuristic
        self.f = f
        self.parent = parent
    

    def set_parent(self, parent):
        self.parent = parent
    
    def set_f(self, f):
        self.f = f
    
    def get_index(self):
        return self.index
    
    def get_f(self):
        return self.f

    def get_h(self):
        return self.heuristic

    def get_G(self):
        return self.f-self.heuristic

    def get_Parent(self):
        return self.parent

    def __str__(self):
        return "index: "+ self.index + " heuristic: " + self.heuristic + " F: " + self.f + " parent: " + self.parent 


def checkIntersect(pos, wp):
    # for wall in walls:
    #     if intersect.doIntersect(line, wall):
    #         return False
    if intersect.get_intersect(pos[0], pos[1], wp[0], wp[1]) < 1:
       return False
    return True

def a_star(start_node, goal_node, list_nodes):
    open_list = []  # creat the open list
    closed_list = []  # create the closed list
    h_start = math.floor(dist(list_nodes[start_node], list_nodes[goal_node]))  # heuristic of the starting node
    start = Node(start_node, h_start, h_start, None)  # create the starting node object
    heapq.heappush(open_list, (start.get_f(), start))  # push the starting node in the heap
    while len(open_list) > 0:  # until the heap is not empty...
        current_node = (heapq.heappop(open_list))[1]  # pop the node with the minimum f
        if current_node.get_index() == goal_node:  # if the current node is the goal node...
            closed_list = np.append(closed_list, current_node)  # push the current node in closed list and break
            break
        else:
            closed_list = np.append(closed_list, current_node)  # push the current node in the closed list
        # discover the neighbors of the current node
        for i in range(len(list_nodes)):
            ver = next((x for x in closed_list if x.get_index() == i), None)
            if checkIntersect(list_nodes[current_node.get_index()], list_nodes[i]) and (ver is None):
                in_open = False
                # calculate the f of the neighbor passing by the current node
                h_temp = math.floor(dist(list_nodes[i], list_nodes[goal_node]))
                f_temp = h_temp+dist(list_nodes[current_node.get_index()],list_nodes[i])+current_node.get_G()
                for j in range(len(open_list)):
                    if open_list[j][1].get_index() == current_node.get_index():  # if the neighbor is in the open list..
                        in_open = True
                        if open_list[j][0] > f_temp:  # ... and the saved f is higher than the current one
                            np.delete(open_list,j)  # delete from the open list...
                            updated_node = Node(i,h_temp,f_temp,current_node)
                            # ... and push the updated one
                            heapq.heappush(open_list,(updated_node.get_f(), updated_node))

                if in_open == False:  # if the neighbor is not in the open list...
                    new_node = Node(i,h_temp,f_temp,current_node)
                    heapq.heappush(open_list, (f_temp, new_node))  # ... and push the node in the open list
    logMessage("%s" %closed_list)
    path = createPath(closed_list[len(closed_list)-1])
    return path


def createPath(node):
    path = []
    parent = node.get_Parent()
    if parent is not None:
        path = createPath(parent)
    path = np.append(path, node)
    return path

def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1, p2)))


def doBehavior(distance, marsData, waypoints, walls, pos):
    #print "doBehavior: " + str(pos[0])
    """
    for wp in waypoints:
        if checkIntersect(pos,wp):
             #print str(pos[0]) + ":" + str(pos[1]) + " to " + str(wp[0]) + ":" + str(wp[1]) + ": no intersection"
             appendLines("path", pos[0], pos[1], 0.5)
             appendLines("path", wp[0], wp[1], 0.5)


#    message = "sensor:"
    """
    goal = [4, 4]
    start = [pos[0], pos[1]]
    waypoints = np.append(waypoints, [start], axis=0)
    waypoints = np.append(waypoints, [goal], axis=0)
    lenght = len(waypoints)
    logMessage("%s" %waypoints)
    path = a_star(lenght-2, lenght-1, waypoints)
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)

    for i in range(len(path)-1):
        appendLines("path", waypoints[path[i].get_index()][0], waypoints[path[i].get_index()][1], 0.5)
        appendLines("path", waypoints[path[i+1].get_index()][0], waypoints[path[i+1].get_index()][1], 0.5)

    return

