from mars_interface import *
import random
import camera_data_acquisition
import numpy as np
import time

global current_behavior

# config variables
high = 120
width = 160
pixelComponents = 4
max_high_ball = 60
min_high_ball = 23
max_high_pole = 120
min_high_pole = 65

range_min_red = [150, 40, 40]
range_max_red = [256, 130, 130]

range_min_green = [40, 200, 40]
range_max_green = [140, 256, 140]

range_min_blue = [20, 20, 150]
range_max_blue = [100, 100, 256]

range_min_yellow = [135, 135, 10]
range_max_yellow = [256, 256, 70]

# global variables
muchRight = 0.
muchLeft = 0.
onlyRight = 1
readyToOrbit = False
lenghtBall = 0
closer = False
numPixels = 0
find_pole = False
right_direction = False
color = "yellow"
kicking = False
tempkicking = 0
tempwaiting = 0


def initialBehavior():
    global current_behavior
    current_behavior = 0


def pointTurn():
    left_cmd = 0.
    right_cmd = -3.
    return left_cmd, right_cmd


def slowPointTurn():
    left_cmd = 1.0
    right_cmd = -1.0
    return left_cmd, right_cmd


def isColorBall(pixel, color):
    global range_max_red, range_max_blue, range_max_green, range_max_yellow
    range_min = [0, 0, 0]
    range_max = [256, 256, 256]
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
        range_max = range_max_yellow
    pixel = [pixel[0] * 255, pixel[1] * 255, pixel[2] * 255]
    if pixel[0] > range_min[0] and pixel[0] < range_max[0] and pixel[1] > range_min[1] and pixel[1] < range_max[1] \
            and pixel[2] > range_min[2] and pixel[2] < range_max[2]:
        return True
    else:
        return False


def isColorPole(pixel, color):
    range_min_red = [230, 40, 40]
    range_max_red = [256, 80, 80]

    range_min_green = [40, 150, 40]
    range_max_green = [180, 256, 180]

    range_min_blue = [20, 20, 130]
    range_max_blue = [100, 130, 256]

    range_min_yellow = [120, 120, 10]
    range_max_yellow = [256, 256, 150]

    range_min = [0, 0, 0]
    range_max = [256, 256, 256]
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
        range_max = range_max_yellow
    pixel = [pixel[0] * 255, pixel[1] * 255, pixel[2] * 255]
    if pixel[0] > range_min[0] and pixel[0] < range_max[0] and pixel[1] > range_min[1] and pixel[1] < range_max[1] \
            and pixel[2] > range_min[2] and pixel[2] < range_max[2]:
        return True
    else:
        return False


def pixelNumber(camera, shape, color, colorFunction):
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
        left_bound = (width / 2 - 20 - 1) * 4
    else:
        max_high = high
        min_high = 0
    middle = (width / 2 - 1) * 4
    pixels = camera_data_acquisition.cameraData[camera][
             width * pixelComponents * min_high:width * pixelComponents * max_high]
    slice_high = max_high - min_high
    rx = 0
    lx = 0
    for j in range(slice_high):
        i = left_bound + 640 * j
        k = middle + 640 * j
        current_middle = middle + 640 * j
        while i < current_middle:
            if colorFunction(pixels[i:i + 3], color):
                lx += 1
            if colorFunction(pixels[k:k + 3], color):
                rx += 1
            i += pixelComponents
            k += pixelComponents
    return lx, rx


def _approachBall(color_of_ball):
    global numPixels
    global closer
    global onlyRight
    direction = None
    lx, rx = pixelNumber("cam0", "ball", color_of_ball, isColorBall)
    logMessage(str(rx + lx))
    numPixels = rx + lx
    onlyRight = rx
    if color_of_ball == "red":
        limit = 1000
    elif color_of_ball == "green":
        limit = 1000
    elif color_of_ball == "blue":
        limit = 900
    else:
        limit = 1000
    if numPixels > limit:
        direction = "stop"
        logMessage("NumPixelAllArrivo %s" % numPixels)
        closer = True
    elif lx > rx + 50:
        direction = "left"
    elif rx > lx + 50:
        direction = "right"
    elif (lx > 0 or rx > 0):
        direction = "straight"
    return direction


def _orbitalWalk(color_of_ball):
    global numPixels
    global readyToOrbit
    global left_cmd
    global right_cmd
    left_cmd = 0
    right_cmd = 0
    # time.sleep(0.080)
    lx, rx = pixelNumber("cam1", "ball", color_of_ball, isColorBall)

    logMessage("Pixeldestra: %s" % rx)
    logMessage("Pixelsinistra: %s" % lx)

    totali = lx + rx
    logMessage("PixelTotali: %s" % totali)

    if lx + rx <= numPixels and not readyToOrbit:
        direction = "right"
    else:
        if not readyToOrbit:
            logMessage("COMINCIO A RUOTARE INTORNO ALLA PALLA")
            readyToOrbit = True

        if abs(totali - numPixels) <= 75:
            if rx > onlyRight:
                logMessage("Mi allontano")
                direction = "orbitRight"
                muchRight = (rx - onlyRight) / (onlyRight + 1)
            else:
                logMessage("Rientro")
                direction = "orbitLeft"
                muchLeft = (rx - onlyRight) / (onlyRight + 1)
        else:
            if totali >= numPixels:
                logMessage("Mi allontano")
                direction = "orbitRight"
                muchRight = (totali - numPixels) / (totali + 1)
            if totali < numPixels:
                logMessage("Mi rientro")
                direction = "orbitLeft"
                muchLeft = (totali - numPixels) / (totali + 1)
    return direction


def detectPole(color_of_pole, cam):
    global find_pole
    global readyToOrbit
    global right_direction
    lx, rx = pixelNumber(cam, "pole", color_of_pole, isColorPole)
    logMessage("lxpole: %s" % lx)
    logMessage("rxpole: %s" % rx)
    if (rx > 0 or lx > 0 and cam == "cam1"):
        find_pole = True
    elif (rx > 0 or lx > 0 and cam == "cam0"):
        right_direction = True


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
        left_cmd = -2.8
        right_cmd = -2.3 + (muchRight / 2.3)
    elif direction == "orbitLeft":
        left_cmd = -2.1 - (muchLeft / 2.1)
        right_cmd = -2.8
    elif direction == "orbit":
        left_cmd = -1.
        right_cmd = -1.
    return left_cmd, right_cmd


def testBehavior():
    cam = camera_data_acquisition.cameraData
    max_high = 60
    min_high = 20
    horizontal = np.copy(cam["cam0"][640 * min_high:640 * max_high])
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
        logMessage("Switching to behavior: " + str(behavior))
        current_behavior = behavior
    if (current_behavior == 3):
        left_cmd = -5.0
        right_cmd = -5.0
    elif (current_behavior == 1):
        left_cmd, right_cmd = approachBall("red")
    elif (current_behavior == 2):
        left_cmd, right_cmd = orbitRedBall()
    elif (current_behavior == 0):
        left_cmd, right_cmd = storeBalls()

    return (left_cmd, right_cmd)


def storeBalls():
    global find_pole
    global right_direction
    global numPixels
    global closer
    global color
    global readyToOrbit
    global kicking
    global tempkicking
    global tempwaiting

    if kicking == True:
        # routine se sto kickando

        # Continuo ad andare avanti a 100 un po
        if tempkicking <= 100:
            tempkicking = tempkicking + 1
            left_cmd = -100.
            right_cmd = -100.

        else:
            if tempwaiting <= 400:
                tempwaiting = tempwaiting + 1
                left_cmd = -1.
                right_cmd = 1.
            else:
                left_cmd = 0.
                right_cmd = 0.
                kicking = False

    else:
        if color is None:
            color = selectColorBall()
        logMessage(color)
        if closer == False:
            left_cmd, right_cmd = approachBall(color)
            lx, rx = pixelNumber("cam0", "ball", color, isColorBall)
            catchedPixel = rx + lx
            if catchedPixel == 0:
                readyToOrbit = False
                closer = False
                find_pole = False
                right_direction = False
                color = None
            logMessage("approaching ball")
        else:
            direction = _orbitalWalk(color)
            left_cmd, right_cmd = move(direction)
            logMessage("Orbiting")
            if readyToOrbit:
                detectPole(color, "cam1")
                logMessage("orbiting searching for a pole")
                if (find_pole and not right_direction):
                    detectPole(color, "cam0")
                    logMessage("right angle found searching the pole")
                    left_cmd, right_cmd = slowPointTurn()
                if (find_pole and right_direction):
                    left_cmd = -100.0
                    right_cmd = -100.0
                    lx, rx = pixelNumber("cam0", "ball", color, isColorBall)
                    catchedPixel = rx + lx
                    logMessage("kicking the ball")
                    kicking = True
                    tempkicking = 0
                    tempwaiting = 0
                    readyToOrbit = False
                    closer = False
                    find_pole = False
                    right_direction = False
                    color = None
    return left_cmd, right_cmd


def selectColorBall():
    lxRed, rxRed = pixelNumber("cam0", "ball", "red", isColorBall)
    lxGreen, rxGreen = pixelNumber("cam0", "ball", "green", isColorBall)
    lxBlue, rxBlue = pixelNumber("cam0", "ball", "blue", isColorBall)
    lxYellow, rxYellow = pixelNumber("cam0", "ball", "yellow", isColorBall)
    colors = [[lxRed + rxRed, "red"], [lxGreen + rxGreen, "green"], [lxBlue + rxBlue, "blue"],
              [lxYellow + rxYellow, "yellow"]]
    colors.sort(key=lambda x: x[0], reverse=True)
    return colors[0][1]


def orbitRedBall():
    if closer == False:
        left_cmd, right_cmd = approachBall("red")
    else:
        direction = _orbitalWalk("red")
        left_cmd, right_cmd = move(direction)
    return left_cmd, right_cmd


def approachBall(color):
    left_cmd, right_cmd = pointTurn()
    direction = _approachBall(color)
    if direction is not None:
        left_cmd, right_cmd = move(direction)
    return left_cmd, right_cmd