from mars_interface import *
import random
import camera_data_acquisition
import numpy as np

global current_behavior

high = 120
width = 160
find_pole = False
pixelComponents = 4
max_high_ball = 80
min_high_ball = 10
max_high_pole = 120
min_high_pole = 80
readyToOrbit = False
lenghtBall = 0
closer = False
numPixels = 0

range_min_red = [150, 40, 40]
range_max_red = [255, 110, 130]

range_min_green = [200, 40, 40]
range_max_green = [255, 70, 70]

range_min_blue = [200, 40, 40]
range_max_blue = [255, 70, 70]

range_min_yellow = [200, 40, 40]
range_max_yellow = [255, 70, 70]


def initialBehavior():
    global current_behavior
    current_behavior = 0

def pointTurn():
    left_cmd  = 0.
    right_cmd = -3.
    return left_cmd, right_cmd

def isColor(pixel, color):
    global range_max_red, range_max_blue, range_max_green, range_max_yellow
    range_min = [0,0,0]
    range_max = [255,255,255]
    if color == "red":
        range_min = range_min_red
        range_max = range_max_red
    elif color == "green":
        range_min = range_min_green
        range_max = range_max_green
    elif color == "blue":
        range_min = range_min_blue
        range_max = range_max_blue
    elif color == "yellow":
        range_min = range_min_yellow
        range_max = range_min_yellow
    pixel = [pixel[0]*255, pixel[1]*255, pixel[2]*255]
    if pixel[0] >range_min[0] and pixel[0] < range_max[0] and pixel[1] > range_min[1] and pixel[1] < range_max[1]\
        and pixel[2] > range_min[2] and pixel[2] < range_max[2]:
        return True
    else:
        return False

def sidePixelNumber(camera, shape, color):
    global width, high
    global pixelComponents
    global min_high_ball, max_high_ball
    global min_high_pole, max_high_pole

    if shape == "ball":
        max_high = max_high_ball
        min_high = min_high_ball
        left_bound = 49 * 4
    elif shape == "pole":
        max_high = max_high_pole
        min_high = min_high_pole
        left_bound = (width/2 - 25 - 1) * 4
    else:
        max_high = high
        min_high = 0
    middle = (width/2 - 1) * 4
    pixels = camera_data_acquisition.cameraData[camera][width*pixelComponents*min_high:width*pixelComponents*max_high]
    slice_high = max_high - min_high
    rx = 0
    lx = 0
    for j in range(slice_high):
        i = left_bound + 640 * j
        k = middle + 640 * j
        current_middle = middle + 640 * j
        while i < current_middle:
            if isColor(pixels[i:i + 3], color):
                lx += 1
            if isColor(pixels[k:k + 3], color):
                rx += 1
            i += pixelComponents
            k += pixelComponents
    return lx, rx

def approachBall(color_of_ball):
    global numPixels
    global closer
    direction = None
    lx, rx = sidePixelNumber("cam0", "ball", color_of_ball)
    numPixels = rx + lx
    if numPixels > 920:
        direction = "stop"
        closer = True
    elif lx > rx+50:
        direction = "left"
    elif rx > lx+50:
        direction = "right"
    elif (lx > 0 or rx > 0):
        direction = "straight"
    return direction

def orbitalWalk(color_of_ball):
    global numPixels
    global readyToOrbit
    lx, rx = sidePixelNumber("cam1", "ball", color_of_ball)
    if abs(numPixels - (rx+lx)) > 50 and not readyToOrbit:
        direction = "right"
    else:
        if not readyToOrbit:
            readyToOrbit = True
        if numPixels < rx + lx:
            direction = "orbitRight"
        elif numPixels >= rx + lx:
            direction = "orbitLeft"
        else:
            direction = "orbit"
    return direction

def detectPole(color_of_pole):
    global find_pole
    global readyToOrbit
    lx, rx = sidePixelNumber("cam1", "pole", color_of_pole)
    logMessage("lxpole: %s" % lx)
    logMessage("rxpole: %s" % rx)
    if (rx > 0 or lx > 0):
        find_pole = True
    
def move(direction):
    right_cmd = -5.
    left_cmd = -5.
    if direction == "right":
        right_cmd = 0.
    elif direction == "left":
        left_cmd = 0.
    elif direction == "stop":
        left_cmd = 0.
        right_cmd = 0.
    elif direction == "orbitRight":
        left_cmd = -4.
        right_cmd = -3.4
    elif direction == "orbitLeft":
        left_cmd = -3.4 
        right_cmd = -4. 
    elif direction == "orbit":
        left_cmd = -1.
        right_cmd = -1.
    return left_cmd, right_cmd

def testBehavior():
    cam = camera_data_acquisition.cameraData
    max_high = 60
    min_high = 20
    horizontal = np.copy(cam["cam0"][640*min_high:640*max_high])
    filename = "prova.ppm"
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
    global closer
    global current_behavior
    (left_cmd, right_cmd) = (0.0, 0.0)
    behavior = marsData["Config"]["Robot"]["behavior"]
    if (behavior != current_behavior):
        logMessage("Switching to behavior: "+str(behavior))
        current_behavior = behavior
    if (current_behavior == 0):
        left_cmd = -5.0
        right_cmd = -5.0
    elif (current_behavior==1):
        left_cmd,right_cmd = pointTurn()
        direction = approachBall("red")
        if direction is not None:
            left_cmd, right_cmd = move(direction)
    elif (current_behavior == 2):
        if closer == False:
            left_cmd,right_cmd = pointTurn()
            direction = approachBall("red")
            if direction is not None:
                left_cmd, right_cmd = move(direction)
        else:
            direction = orbitalWalk("red")
            left_cmd, right_cmd = move(direction)
    elif (current_behavior == 3):
        if closer == False:
            left_cmd,right_cmd = pointTurn()
            direction = approachBall("red")
            if direction is not None:
                left_cmd, right_cmd = move(direction)
        else:
            direction = orbitalWalk("red")
            if readyToOrbit:
                detectPole("red")
            if (find_pole):
                direction = "stop"
            left_cmd, right_cmd = move(direction)

    return (left_cmd, right_cmd)