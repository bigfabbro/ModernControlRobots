from mars_interface import *
import random
import astar
import numpy as np

random.seed()
hg = []
right_actuator = 0.
left_actuator = 0.
path = []
update_time = 0.02
wheel_distance = 0.485751326
radius = 0.15
waypoints = None
walls = None
first_time = True
old_path = []
old_left = 0
old_right = 0
avoiding = 0
forced_waypoint = []
prev_goal = []


def updatePath(pose, goal):
     global path, waypoints, walls
     path = astar.run(waypoints, walls, np.array([pose[0], pose[1]]),
                      np.array(goal))
     clearLines("path")
     configureLines("path", 5, 0.8, 0.6, 0.2)
     for i in range(len(path)-1):
          appendLines("path", path[i][0], path[i][1], 0.45)
          appendLines("path", path[i+1][0], path[i+1][1], 0.45)


def autonomousDrive(pose, goal):
    global path
    rob_pos = [pose[0], pose[1]]
    updatePath(pose, goal)
    scaled_wp = np.subtract(path[-2], rob_pos)
    vers = [np.cos(pose[2]), np.sin(pose[2])]
    #F_att = k_att * np.linalg.norm(scaled_wp)
    norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
    angle = np.arccos(np.dot(vers, scaled_wp)/norm)
    sin = np.cross(vers, scaled_wp)/norm
    if sin < 0 and angle > 0.05:
        la = 1.
        ra = -1.
    elif sin > 0 and angle > 0.05:
        la = -1.
        ra = 1.
    else:
        la = 2.
        ra = 2.
    
    return la, ra

def obstacleAvoidance(pose):
    global path, old_path, old_left, old_right, avoiding, forced_waypoint
    rob_pos = [pose[0], pose[1]]


    if avoiding > 0:
        print("AVOIDING")
        path = old_path
        avoiding = avoiding - 1
        scaled_wp = np.subtract(forced_waypoint, rob_pos)
        vers = [np.cos(pose[2]), np.sin(pose[2])]
        #F_att = k_att * np.linalg.norm(scaled_wp)
        norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
        angle = np.arccos(np.dot(vers, scaled_wp)/norm)
        if angle > 0.1:
            sin = np.cross(vers, scaled_wp)/norm
            if sin < 0:
                la = 2.
                ra = 0.5
            if sin > 0:
                la = 0.5
                ra = 2.
            return la, ra, True
        else:
            return 0., 0., True
        

    elif np.array_equal(path[-2], old_path[-2]) != True:
        print ("CAMBIO PATH")
        dist = np.sqrt(np.sum(np.square(np.subtract(rob_pos, old_path[-2]))))
        if  dist > 0.15:
            path = old_path
            scaled_wp = np.subtract(path[-2], rob_pos)
            vers = [np.cos(pose[2]), np.sin(pose[2])]
            #F_att = k_att * np.linalg.norm(scaled_wp)
            norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
            angle = np.arccos(np.dot(vers, scaled_wp)/norm)
            sin = np.cross(vers, scaled_wp)/norm
            if sin < 0 and angle > 0.1:
                la = 1.
                ra = -1.
            elif sin > 0 and angle > 0.1:
                la = -1.
                ra = 1.
            else:
                la = 2.
                ra = 2.
            
            return la, ra, True
        else:
            forced_waypoint = path[-2]
            scaled_wp = np.subtract(path[-2], rob_pos)
            vers = [np.cos(pose[2]), np.sin(pose[2])]
            #F_att = k_att * np.linalg.norm(scaled_wp)
            norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
            angle = np.arccos(np.dot(vers, scaled_wp)/norm)
            if angle > 0.1:
                avoiding = 50
                sin = np.cross(vers, scaled_wp)/norm
                if sin < 0:
                    la = 3.
                    ra = 2.
                if sin > 0:
                    la = 2.
                    ra = 3.
                return la, ra, True
            else:
                return 0., 0., False
            
    else:
        return 0.,0.,False

def doBehavior(distance, marsData, pose, goal):
    global right_actuator, left_actuator, first_time, old_path, path, old_left, old_right, prev_goal
    # pose[0] = x
    # pose[1] = y
    # pose[2] = z
    # goal[0] = x
    # goal[1] = y
    
    if first_time or np.array_equal(prev_goal, goal) != True :
        updatePath(pose, goal)
        first_time = False

    prev_goal = goal
    old_path = path
        
    old_left = left_actuator
    old_right = right_actuator
    joystickLeft = marsData["Config"]["VirtualJoystick"]["x"]
    joystickRight = marsData["Config"]["VirtualJoystick"]["y"]
    left_actuator, right_actuator = autonomousDrive(pose, goal)

    la, ra, change = obstacleAvoidance(pose)
    if change == True:
        left_actuator, right_actuator = la,ra


    #print (pose)
    #behavior = marsData["Config"]["Robotik2"]["behavior"]

    # if timing(1):
    #     message = "sensor:"
    #     logMessage(message)
    return



