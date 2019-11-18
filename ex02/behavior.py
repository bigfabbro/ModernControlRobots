from mars_interface import *
import random
import camera_data_acquisition
import numpy as np

global current_behavior

def initialBehavior():
    global current_behavior
    current_behavior = 0

def pointTurn():
    left_cmd  = 0.
    right_cmd = -2.
    return left_cmd, right_cmd

def detectRedBall():
    cam_name = camera_data_acquisition.CAMERA_NAMES
    cam = camera_data_acquisition.cameraData
    range_min = [190, 50, 50]
    range_max = [255,80,80]
    max_high = 60
    min_high = 20
    pixels = np.copy(cam["cam0"][640*min_high:640*max_high])
    find = False
    i = 0
    while i < len(pixels):
        pixel = [pixels[i]*255, pixels[i+1]*255, pixels[i+2]*255]
        if pixel[0] > 190 and pixel[0]< 255 and pixel[1] > 50 and pixel[1] < 80 and pixel[2] > 50 and pixel[2] < 80:
            find = True
            break
        i += 4
    return find

def computeRedBallHorizontalPosition():




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
        detectRedBall()
    return (left_cmd, right_cmd)
