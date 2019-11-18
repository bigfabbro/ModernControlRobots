from mars_interface import *
import random
import camera_data_acquisition
import numpy as np

global current_behavior

range_min = [235, 40, 40]
range_max = [255, 60, 60]
max_high = 80
min_high = 10
find = False
lenghtBall = 0


def convertInPixel(camera_name):
    cam = camera_data_acquisition.cameraData[camera_name]
    pixels = []
    i = 0
    while i < len(cam):
        pixel = [cam[i]*255, cam[i+1]*255, cam[i+2]*255]
        pixels = np.append(pixels, pixel)
        i += 4
    return pixels

def giveBord(pixels):
    left_bord = pixels[pixels % 160 == 0]
    return left_bord

def initialBehavior():
    global current_behavior
    current_behavior = 0

def pointTurn():
    left_cmd  = 0.
    right_cmd = -1.
    return left_cmd, right_cmd

def isRed(pixel):
    pixel = [pixel[0]*255, pixel[1]*255, pixel[2]*255]
    if pixel[0] >range_min[0] and pixel[0] < range_max[0] and pixel[1] > range_min[1] and pixel[1] < range_max[1]\
        and pixel[2] > range_min[2] and pixel[2] < range_max[2]:
        return True
    else:
        return False

def centering():
    pixelsRight = camera_data_acquisition.cameraData["cam1"][640 * min_high:640 * max_high]
    pixelsLeft = camera_data_acquisition.cameraData["cam0"][640 * min_high:640 * max_high]
    left_bound = 49 * 4
    right_bound = 109 * 4
    middle = 79*4
    high = max_high - min_high
    rx = 0
    lx = 0
    for j in range(high):
        i = left_bound + 640 * j
        k = middle + 640 * j
        current_middle = middle + 640 * j
        while i < current_middle:
            if isRed(pixelsLeft[i:i+3]):
                lx += 1
            if isRed(pixelsLeft[k:k+3]):
                rx += 1
            i += 4
            k += 4
    if rx + lx > 500:
        direction = "stop"
    elif abs(lx - rx) < 5000 and (lx > 0 and rx > 0):
        direction = "straight"
    elif lx > rx:
        direction = "left"
    else:
        direction = "right"
    return  direction




def moveToBall(direction):
    right_cmd = -5.
    left_cmd = -5.
    if direction == "right":
        right_cmd = 0.
    elif direction == "left":
        left_cmd = 0.
    elif direction == "stop":
        left_cmd = 0.
        right_cmd = 0.
    return left_cmd, right_cmd

def computeRedBallHorizontalPosition():
    global lenghtBall
    lenghtBall += 1

def testBehavior():
    cam = camera_data_acquisition.cameraData
    max_high = 60
    min_high = 20
    horizontal = np.copy(cam["cam0"][640*min_high:640*max_high])
    filename = "prova.ppm"
    logMessage("lungh %s" % len(horizontal))
    with open(filename, "w") as f:
        f.write("P3\n")
        f.write(str(160) + " " + str(40) + "\n")
        f.write("255\n")
        for y in range(40):
            yy = 40 - 1 - y
            for x in range(160):
                # Each pixel has three components Red Green and Blue
                f.write(str(int(horizontal[yy * 160 * 4 + x * 4] * 255)) + " ")
                f.write(str(int(horizontal[yy * 160 * 4 + x * 4 + 1] * 255)) + " ")
                f.write(str(int(horizontal[yy * 160 * 4 + x * 4 + 2] * 255)) + " ")
            f.write("\n")

def doBehavior(marsData):
    global current_behavior
    (left_cmd, right_cmd) = (0.0, 0.0)
    behavior = marsData["Config"]["Robot"]["behavior"]
    if (behavior != current_behavior):
        logMessage("Switching to behavior: "+str(behavior))
        current_behavior = behavior
    if (current_behavior == 0):
        left_cmd = -2.0
        right_cmd = -2.0
    elif (current_behavior==1):
        left_cmd,right_cmd = pointTurn()
        direction = centering()
        logMessage("%s" % direction)
        if direction is not None:
            left_cmd, right_cmd = moveToBall(direction)
    return (left_cmd, right_cmd)