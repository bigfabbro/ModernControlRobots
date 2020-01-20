from mars_interface import *
import random
import astar
import numpy as np

random.seed()

right_actuator = 0.
left_actuator = 0.
path = []
waypoints = None
walls = None
first_time = True

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
    if np.sqrt(np.sum(np.square(np.subtract(rob_pos, path[-2])))) < 0.4:
        updatePath(pose, goal)
    scaled_wp = np.subtract(path[-2], rob_pos)
    vers = [np.cos(pose[2]), np.sin(pose[2])]
    norm = np.linalg.norm(scaled_wp) * np.linalg.norm(vers)
    angle = np.arccos(np.dot(vers, scaled_wp)/norm)
    sin = np.cross(vers, scaled_wp)/norm
    if sin < 0 and angle > 0.2:
        return 2., 1.
    elif sin > 0 and angle > 0.2:
        return 1., 2.
    else:
        return 3., 3.

def obstacleAvoidance(distance):

    if distance[0] < 0.5 and distance[7] < 0.5:
        la = -2.
        ra = -2.
        oa = True
    elif distance[11] < 0.5 and distance[12] < 0.5:
        la = 2.
        ra = 2.
        oa = True




def normalizeAngle(angle):
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < np.pi:
        angle += 2*np.pi



def doBehavior(distance, marsData, pose, goal):
     global right_actuator, left_actuator, first_time
     # pose[0] = x
     # pose[1] = y
     # pose[2] = z
     # goal[0] = x
     # goal[1] = y
     goal = [-5, 4]
     print (distance)
     if first_time:
         updatePath(pose, goal)
         first_time = False
     joystickLeft = marsData["Config"]["VirtualJoystick"]["x"]
     joystickRight = marsData["Config"]["VirtualJoystick"]["y"]
     #updatePath(pose, goal)
     #left_actuator = joystickLeft
     #right_actuator = joystickRight
     left_actuator, right_actuator = autonomousDrive(pose, goal)
     ob_return = obstacleAvoidance(distance)
     if ob_return[2]:
         left_actuator = ob_return[0]
         right_actuator = ob_return[1]
     #print (pose)
     #behavior = marsData["Config"]["Robotik2"]["behavior"]

     # if timing(1):
     #     message = "sensor:"
     #     logMessage(message)
     return

