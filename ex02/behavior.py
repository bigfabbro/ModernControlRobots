from mars_interface import *
import random
import time
import camera_data_acquisition
import numpy as np

global current_behavior

high = 120
width = 160
pixelComponents = 4

max_high_ball = 60
min_high_ball = 24
max_high_pole = 110
min_high_pole = 70

numPixels = 0
hitting = 0
hit = False
readyToOrbit = False
readyToHit = False
find_pole = False
closer = False
start = True

colours = ["red", "green", "blue", "yellow"]

# configuration parameters in this order: R_min, G_min, B_min, R_max, G_max, B_max, dist_appr, pixel_rot
red = [150, 40, 40, 255, 130, 130, 230, 40, 40, 255, 80, 80, 900, 40]
blue = [20, 20, 135, 100, 100, 255, 40, 40, 90, 120, 140, 255, 750, 50]
green = [40, 200, 40, 140, 255, 140, 40, 150, 40, 180, 255, 180, 900, 50]
yellow = [135, 135, 10, 255, 255, 70, 120, 120, 10, 255, 255, 150, 900, 50]
dist_appr = 0
pixel_rot = 0

range_min_ball = None
range_max_ball = None
range_min_pole = None
range_max_pole = None


def initialBehavior():
    global current_behavior
    current_behavior = 0

def setParameters(colour):
    global range_min_ball, range_max_ball, range_max_pole, range_min_pole
    global dist_appr, pixel_rot
    global red, blue, green, yellow
    global readyToOrbit, readyToHit, closer, hit, hitting, find_pole, start

    if colour == "blue":
        array = blue
    elif colour == "green":
        array = green
    elif colour == "yellow":
        array = yellow
    else:
        array = red

    range_min_ball = array[0:3]
    range_max_ball = array[3:6]
    range_min_pole = array[6:9]
    range_max_pole = array[9:12]
    dist_appr = array[12]
    pixel_rot = array[13]


def reset():
    global hitting, hit, readyToHit, readyToOrbit, find_pole, closer, start
    hitting = 0
    hit = False
    readyToOrbit = False
    readyToHit = False
    find_pole = False
    closer = False
    start = True

def isColor(pixel, shape):
    global range_min_ball, range_max_ball, range_min_pole, range_max_pole
    pixel = [pixel[0]*255, pixel[1]*255, pixel[2]*255]
    if shape == "ball":
        if pixel[0] >= range_min_ball[0] and pixel[0] <= range_max_ball[0] and pixel[1] >= range_min_ball[1] and pixel[1] <= range_max_ball[1]\
        and pixel[2] >= range_min_ball[2] and pixel[2] <= range_max_ball[2]:
            return True
        else:
            return False
    else:
        if pixel[0] >= range_min_pole[0] and pixel[0] <= range_max_pole[0] and pixel[1] >= range_min_pole[1]\
        and pixel[1] <=range_max_pole[1] and pixel[2] >= range_min_pole[2] and pixel[2] <= range_max_pole[2]:
            return True
        else:
            return False

def sidePixelNumber(camera, shape, color = None):
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
        left_bound = (width/2 - 5 - 1) * 4
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
            if isColor(pixels[i:i + 3], shape):
                lx += 1
            if isColor(pixels[k:k + 3], shape):
                rx += 1
            i += pixelComponents
            k += pixelComponents
    return lx, rx

def pointTurn():
    left_cmd  = -2.
    right_cmd = 2.
    return left_cmd, right_cmd

def approachBall(lx,rx):
    global numPixels
    global closer
    global onlyRight
    global dist_appr
    if numPixels > dist_appr:
        left_cmd = 0.
        right_cmd = 0.
        closer = True
    elif lx > rx+50:
        left_cmd = 0.
        right_cmd = -5.
    elif rx > lx+50:
        left_cmd = -5.
        right_cmd = 0.
    elif (lx > 0 or rx > 0):
        left_cmd = -5.
        right_cmd = -5.
    return left_cmd, right_cmd

def orbitalWalk(lx,rx):
    global numPixels
    global readyToOrbit
    global pixel_rot

    if lx+rx < numPixels - pixel_rot and not readyToOrbit:
        left_cmd = -1.
        right_cmd = 1.
    else:
        if not readyToOrbit:
            readyToOrbit = True
        if lx > rx:
            left_cmd = -2.1
            right_cmd = -2.8 - (lx + rx )/numPixels
        else:
            left_cmd = -2.8 - (lx + rx)/ numPixels
            right_cmd = -2.3
    return left_cmd, right_cmd

def hitBall(lx, rx):
    global readyToHit, hit
    global numPixels
    if lx + rx < numPixels - pixel_rot and not readyToHit:
        left_cmd = 1.
        right_cmd = -3.
    else:
        if not readyToHit:
            readyToHit = True
        left_cmd = -8.
        right_cmd = -8.
        hit = True
    return left_cmd, right_cmd

def detectPole(lp, rp):
    global find_pole
    if (rp >0 or lp > 0):
        find_pole = True

def selectColorBall():
    global colours
    values = []
    for colour in colours:
        setParameters(colour)
        lx, rx = sidePixelNumber("cam0", "ball")
        values.append([lx + rx, colour])
    values.sort(key=lambda x: x[0], reverse=True)
    logMessage("%s" % values)
    return values[0]


def approachRedBall():
    global start
    global numPixels
    if start:
        setParameters("red")
        start = False
    left_cmd, right_cmd = pointTurn()
    lx, rx = sidePixelNumber("cam0", "ball")
    numPixels = rx + lx
    if numPixels > 0:
        left_cmd, right_cmd = approachBall(lx, rx)
    return left_cmd, right_cmd

def orbitRedBall():
    global closer
    if not closer:
        left_cmd, right_cmd = approachRedBall()
    else:
        lx, rx = sidePixelNumber("cam1", "ball")
        left_cmd, right_cmd = orbitalWalk(lx, rx)
    return left_cmd, right_cmd

def storeTheBalls():
    global start, find_pole, hit
    global hitting, numPixels
    if start:
        colour = selectColorBall()
        if colour[0] > 5:
            setParameters(colour[1])
            start = False
        else:
            left_cmd = -2.
            right_cmd = 2.
    if not closer:
        lx, rx = sidePixelNumber("cam0", "ball")
        left_cmd, right_cmd = pointTurn()
        numPixels = rx + lx
        if (numPixels > 0):
            left_cmd, right_cmd = approachBall(lx, rx)
    else:
        if not find_pole:
            lx, rx = sidePixelNumber("cam1", "ball")
            left_cmd, right_cmd = orbitalWalk(lx, rx)
        if readyToOrbit:
            lp, rp = sidePixelNumber("cam1", "pole")
            detectPole(lp, rp)
            if find_pole:
                lx, rx = sidePixelNumber("cam0", "ball")
                left_cmd, right_cmd = hitBall(lx, rx)
                if hit:
                    hitting += 1
                    if hitting < 250:
                        left_cmd = -8.
                        right_cmd = -8.
                    elif hitting < 400:
                        left_cmd = -2.
                        right_cmd = 2.
                    else:
                        reset()
    return left_cmd, right_cmd

def doBehavior(marsData):
    global closer, hitting
    global current_behavior, numPixels
    global readyToOrbit, readyToHit
    global start, hit
    (left_cmd, right_cmd) = (0.0, 0.0)
    behavior = marsData["Config"]["Robot"]["behavior"]
    if behavior != current_behavior:
        logMessage("Switching to behavior: "+str(behavior))
        current_behavior = behavior
    if current_behavior == 0:
        left_cmd = -5.0
        right_cmd = -5.0
    elif current_behavior == 1:
       left_cmd, right_cmd = approachRedBall()
    elif current_behavior == 2:
      left_cmd, right_cmd = orbitRedBall()
    elif (current_behavior == 3):
        left_cmd, right_cmd = storeTheBalls()
    return (left_cmd, right_cmd)


