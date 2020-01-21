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

class HistrogramGrid:

    def __init__(self, col, row, map, scale):
        self.col = col
        self.row = row
        self.scale = scale
        self.grid = np.repeat([np.zeros(col)], row, axis=0)
        f = open(map, "r")
        for line in f:
            coordinates = [float(x) for x in line.split(";")]
            coordinates = np.dot(coordinates, scale)
            coordinates[1] *= -1
            coordinates[3] *= -1
            coordinates = coordinates + [col/2, row/2, col/2, row/2]
            x = [coordinates[0], coordinates[2]]
            x.sort()
            y = [coordinates[1], coordinates[3]]
            y.sort()
            if x[1] - x[0] == 0:
                for i in range(int(y[1]-y[0])):
                    self.grid[int(y[0])+i][int(coordinates[0])] = 1
            else:
                for j in range(int(x[1] - x[0])):
                    self.grid[int(coordinates[1])][int(x[0])+j] = 1

    def translateCoord(self, coordinates):

        coordinates = np.dot(coordinates, self.scale)
        coordinates[1] *= -1
        coordinates = coordinates + [self.col/2, self.row/2]

        return coordinates

    def forceActiveArea(self, robPos):

        trans_rob_pos = self.translateCoord(robPos)
        active_area_x = [int(trans_rob_pos[0] - 100), int(trans_rob_pos[0]+100)]
        active_area_y = [int(trans_rob_pos[1] - 100), int(trans_rob_pos[1]+100)]
        forces = [0, 0]
        for i in range(200):
            for j in range(200):
                if self.grid[active_area_y[0]+i][active_area_x[0]+j] == 1:
                    direction = np.arctan(-(active_area_y[0]+i - trans_rob_pos[1])/(active_area_x[0]+j - trans_rob_pos[0]))
                    if active_area_x[0]+j > trans_rob_pos[0]:
                        direction -= np.pi
                    magnitude = (111 - np.sqrt((active_area_y[0]+i - trans_rob_pos[1])**2+(active_area_x[0]+j -trans_rob_pos[0])**2))
                    forces[0] += magnitude * np.cos(direction)
                    forces[1] += magnitude * np.sin(direction)
        r = np.sqrt(forces[0]**2 + forces[1]**2)
        theta = np.arctan(forces[1]/forces[0])
        if forces[0] < 0:
            theta += np.pi
        return r, theta


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


def doBehavior(distance, marsData, pose, goal):
     global right_actuator, left_actuator, first_time, hg, update_time, wheel_distance, radius
     # pose[0] = x
     # pose[1] = y
     # pose[2] = z
     # goal[0] = x
     # goal[1] = y
     goal = [-5, 4]
     print (distance)
     if first_time:
         updatePath(pose, goal)
         hg = HistrogramGrid(1600, 1400, "map.txt", 100)
         first_time = False
     joystickLeft = marsData["Config"]["VirtualJoystick"]["x"]
     joystickRight = marsData["Config"]["VirtualJoystick"]["y"]
     #updatePath(pose, goal)
     left_actuator, right_actuator = autonomousDrive(pose, goal)
     magnitude, direction = hg.forceActiveArea([pose[0], pose[1]])
     logMessage("direction: %s" %direction)
     logMessage("magnitude: %s" %magnitude)
     """force_vers = [np.cos(direction), np.sin(direction)]
     pose_vers = [np.cos(pose[2]), np.sin(pose[2])]
     norm = np.linalg.norm(direction) * np.linalg.norm(pose_vers)
     angle = np.arccos(np.dot(pose_vers, force_vers)/norm)
     sin = np.cross(pose_vers, force_vers)/norm
     if sin < 0:
         left_actuator = 2.
         right_actuator = 1.
     elif sin > 0:
         left_actuator = 1.
         right_actuator = 2."""
     #print (pose)
     #behavior = marsData["Config"]["Robotik2"]["behavior"]

     # if timing(1):
     #     message = "sensor:"
     #     logMessage(message)
     return



