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
    #take the robot position, thus the particle's <x,y> (that comes from the particle filter)
    rob_pos = [pose[0], pose[1]]
    #update the path to calculate where to move, using the astar algorithm, starting from the <x,y> of the particle
    updatePath(pose, goal)
    #calculating the coordinate of the closest wayppoint (on the path from astar) with respect to the <x,y> of the particle
    scaled_wp = np.subtract(path[-2], rob_pos)
    #the direction of the robot expressed as <cos,sin>
    vers = [np.cos(pose[2]), np.sin(pose[2])]
    norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
    
    #The angle between the direction of the robot and the closest waypoint on the path;
    #this will be used to calculate where to move in order to reach the waypoint

    #angle is used to build a range in where the robot has to adjust its direction (with respect to the waypoint)
    #sin is used to understand where (if right or left) the waypoint is
    
    angle = np.arccos(np.dot(vers, scaled_wp)/norm)
    sin = np.cross(vers, scaled_wp)/norm

    #rotate on itself to align its direction with the waypoint
    if sin < 0 and angle > 0.1:
        la = 1.
        ra = -1.
    elif sin > 0 and angle > 0.1:
        la = -1.
        ra = 1.
    #if aligned the robot can just go straight
    else:
        la = 2.
        ra = 2.
    
    return la, ra



def obstacleAvoidance(pose):
    global path, old_path, old_left, old_right, avoiding, forced_waypoint
    rob_pos = [pose[0], pose[1]]

    #check if the robot is performing the avoiding routine, if it is, there is nothing more to do
    #during the avoiding routine the robot turn softly on the side of the new waypoint (instead of the autonomous drive, in witch it turns around its pose)
    if avoiding > 0:
        avoiding = avoiding - 1
        scaled_wp = np.subtract(forced_waypoint, rob_pos)
        vers = [np.cos(pose[2]), np.sin(pose[2])]
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
        
    #check if the path is changed (this can be changed only by the autonomous drive)
    elif np.array_equal(path[-2], old_path[-2]) != True:
        dist = np.sqrt(np.sum(np.square(np.subtract(rob_pos, old_path[-2]))))
        #check if the robot is yet too far from to the waypoint of the old path
        #if it is still too far, it has to reach the waypoint before to take another path 
        if  dist > 0.15:
            #force the new proposed path (by the autonomous drive) to be the previous one
            path = old_path

            #perform the same turning routine of the autonomous drive..
            scaled_wp = np.subtract(path[-2], rob_pos)
            vers = [np.cos(pose[2]), np.sin(pose[2])]
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
            #if it is close enough to the waypoint it can start to turn over the next one (rotating on itself to align with the new waypoint to reach)
            #forced_waypoint is used to not allow changing in path during the avoiding routine
            forced_waypoint = path[-2]
            avoiding = 50
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

    return



