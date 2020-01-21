from mars_interface import *
import behavior
import particlefilter as pf
import math
import intersect
import astar
import numpy as np
from euclid import *
import time

distance = np.zeros(16)
pos = Vector3()
direction = 0.0
pointCloudData = {}
t = 0.
walls = None
waypoints = None
path = None
firstUpdate = True
numParticles = 100

def addPointCloudData(name, points):
    global pointCloudData, numParticles
    pointCloudData[name] = points
    logMessage(name)
    if name == "waypoints":
        i = 0
        for waypoint in waypoints:
            points[i*3] = waypoint[0]
            points[i*3+1] = waypoint[1]
            points[i*3+2] = 0.5
            i += 1
    if name == "particles":
        for i in range(numParticles):
            points[i*3] = 0.0
            points[i*3+1] = 0.0
            points[i*3+2] = 0.5
            
def loadPoints(fName):
    array = []
    with open(fName) as f:
        for line in f:
            line = line.strip()
            if len(line) == 0 or line[0] == "#":
                continue
            arrLine = line.split()
            if len(arrLine) == 4:
                array.append([float(arrLine[0]), float(arrLine[1]),
                              float(arrLine[2]), float(arrLine[3])])
            elif len(arrLine) == 2:
                array.append([float(arrLine[0]), float(arrLine[1])])
    return np.array(array)

def init():
    global light_sensor, walls, waypoints, firstUpdate
    firstUpdate = True
    reload(behavior)
    reload(astar)
    reload(pf)
    reload(intersect)
    clearDict()
    #setRunning(True)
    requestSensor("position")
    requestSensor("rotation")
    requestSensor("laser_vl")
    requestSensor("laser_vr")
    requestSensor("laser_hl")
    requestSensor("laser_hr")
    requestConfig("VirtualJoystick", "x")
    requestConfig("VirtualJoystick", "y")
    
    setConfig("Graphics", "showCoords", 0)
    setConfig("Scene", "skydome_enabled", 1)
    setConfig("Simulator", "calc_ms", 40)


    setConfig("Robotik2", "goal/x", 4)
    setConfig("Robotik2", "goal/y", 3)
    requestConfig("Robotik2", "goal/x")
    requestConfig("Robotik2", "goal/y")
    setUpdateTime(10)

    # init point lists
    waypoints = loadPoints("python/waypoints.txt")
    behavior.waypoints = waypoints
    logMessage("loaded " + str(len(waypoints)) + " waypoints")
    createPointCloud("waypoints", len(waypoints))
    createPointCloud("particles", numParticles)
    walls = loadPoints("python/walls.txt")
    behavior.walls = walls
    #intersect.initStaticLines(walls)
    logMessage("loaded " + str(len(walls)) + " walls")
    clearLines("walls")
    clearLines("path")
    for w in walls:
        appendLines("walls", w[0], w[1], 2.4)
        appendLines("walls", w[2], w[3], 2.4)
    configureLines("walls", 3.0, 0, 1, 0)
    # caluculate squared lenth of walls
    return sendDict()

def getVector3(myList):
    v = Vector3()
    if len(myList) == 3:
        v = Vector3(myList[0], myList[1], myList[2])
    return v

def readSensor(marsData):
    global distance, pos, direction
    for i in range(4):
        distance[i] = marsData["Sensors"]["laser_vl"][i]
    for i in range(4):
        distance[4+i] = marsData["Sensors"]["laser_vr"][i]
    for i in range(4):
        distance[8+i] = marsData["Sensors"]["laser_hl"][i]
    for i in range(4):
        distance[12+i] = marsData["Sensors"]["laser_hr"][i]
    pos = getVector3(marsData["Sensors"]["position"])
    direction = math.radians(getVector3(marsData["Sensors"]["rotation"]).z)

def update(marsData):
    global distance, t, pointCloudData, firstUpdate, walls, numParticles
    clearDict()
    readSensor(marsData)

  
    if firstUpdate:
        firstUpdate = False
        configureLines("guess", 5, 0.2, 1.0, 0.2)
        configureLines("debugRays", 2, 0.5, 0.4, 1.0)
        configurePointCloud("waypoints", 8.0, 0.2, 0.6, 0.8)
        configurePointCloud("particles", 4.0, 0.9, 0.3, 0.9)
        # init particle filter
        pf.init(numParticles, 3,
                np.array([-0.5, -0.5, -0.1]), np.array([0.5, 0.5, 0.1]))
        #logMessage(str(pf.population))

    message = "sensor: "
    message += str(direction) + " " + str(pos)

    #intersect.get_intersect(0., 0., 10.0, 0.0)

    pf.sample_and_weight(walls, distance, np.array([behavior.left_actuator*0.15, behavior.right_actuator*0.15]))

    # update particles in mars
    pData = pointCloudData["particles"]
    for i in range(numParticles):
         pData[i*3] = pf.population[i][0]
         pData[i*3+1] = pf.population[i][1]
    
    pf.select_best_trunc()
    clearLines("debugRays")
    
    bestP = pf.population[0]    
    p2 = np.array([math.cos(bestP[2]), math.sin(bestP[2])])
    np.multiply(p2, 0.2, p2)
    np.add(bestP[:2], p2, p2)

    vMeasure = pf.generate_measurement(bestP, walls, True)
    #vMeasure = pf.generate_measurement(np.array([pos[0], pos[1], direction, 1.]), walls, True)
    # logMessage(str(distance))
    # logMessage(str(vMeasure))
    # logMessage(".")

    clearLines("guess")
    appendLines("guess", bestP[0], bestP[1], 0.5)
    appendLines("guess", p2[0], p2[1], 0.5)

    goalx = marsData["Config"]["Robotik2"]["goal/x"]
    goaly = marsData["Config"]["Robotik2"]["goal/y"]
    behavior.doBehavior(distance, marsData, bestP, [goalx, goaly])

    setMotor("motor_links", behavior.left_actuator)
    setMotor("motor_rechts", behavior.right_actuator)
    return sendDict()
