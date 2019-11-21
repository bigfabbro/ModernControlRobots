from mars_interface import *
from euclid import *
import math
from camera_data_acquisition import CameraDataAcquisition, addCameraData
from behavior import initialBehavior, doBehavior


global cda

def init():
    global cda
    clearDict()
    cda = CameraDataAcquisition()
    initialBehavior()
    setConfig("Robot", "behavior", 0)
    requestConfig("Robot", "behavior")
    setRunning(True)
    logMessage("setup python interface")
    return sendDict()

def update(marsData):
    global cda
    clearDict()
    cda.acquire_images()
    (motor_left_cmd, motor_right_cmd) = doBehavior(marsData)
    setMotor("motor_left", motor_left_cmd )
    setMotor("motor_right", motor_right_cmd)
    return sendDict()
