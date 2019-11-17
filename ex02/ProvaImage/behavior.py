from mars_interface import *
from mars_interface import *
from euclid import *
import math
import numpy as np
from camera_data_acquisition import *
import random


global current_behavior, cda, cameraData, red_min, red_max

'''def addCameraData(name, data):
    global cameraData, cameraSize
    cameraData[name] = data
    cameraSize[name] = len(data)/4'''

def initialBehavior():
    global current_behavior
    current_behavior = 0
    
def doBehavior(marsData):
    global current_behavior, cda, red_min, red_max#, cameraData#, cameraSize
    red_min = np.array([230,45,45])
    red_max = np.array([255,90,90])
    (left_cmd, right_cmd) = (0.0, 0.0)
    behavior = marsData["Config"]["Robot"]["behavior"]
    if (behavior != current_behavior):
        logMessage("Switching to behavior: "+str(behavior))
        current_behavior = behavior
    if (current_behavior == 0):
        left_cmd = -2.0
        right_cmd = -2.0
    elif (current_behavior == 1):
        left_cmd = 2.0
        right_cmd = 0.0
        #logMessage())
        cam0_array = np.array(cameraData["cam0"])*255
        cam1_array = np.array(cameraData["cam1"])*255
        cam0_array_reshaped = np.reshape(cam0_array, (19200,4))
        cam1_array_reshaped = np.reshape(cam1_array, (19200,4))
        cam0_RGB = cam0_array_reshaped[9280:11680,:3]
        cam1_RGB = cam1_array_reshaped[9280:11680,:3]
        for pixel in np.arange(2400):
            if ((cam0_RGB[pixel,:] >= red_min).all() and (cam0_RGB[pixel,:] <= red_max).all()):
                logMessage("red found")
                left_cmd = 0.0
                right_cmd = 0.0
                return (left_cmd, right_cmd)
        '''mask1 = cam0_RGB[np.greater_equal(cam0_RGB[:,0],red_min[0])]
        mask2 = mask1[np.less_equal(mask1[:,0],red_max[0])]
        mask3 = cam1_RGB[np.greater_equal(cam1_RGB[:,0],red_min[0])]
        mask4 = mask3[np.less_equal(mask3[:,0],red_max[0])]'''
        '''if ((mask2.size != 0) or (mask4.size != 0)):
            logMessage("red found")'''
    return (left_cmd, right_cmd)
