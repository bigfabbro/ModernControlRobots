from time import clock
from mars_interface import *

CAMERA_NAMES = ["cam0", "cam1"]
STORAGE_PATH = "./"

cameraData = {}
cameraSize = {}

def addCameraData(name, data):
    global cameraData, cameraSize
    cameraData[name] = data
    cameraSize[name] = len(data)/4

class CameraDataAcquisition():


    def __init__(self):
        self.start_times = {}
        for camera_name in CAMERA_NAMES:
            requestCameraSensor(camera_name)
            self.start_times[camera_name] = clock()


    def elapsed_time(self, timer_name, s):
        current_time = clock()
        if current_time > self.start_times[timer_name]+s:
            self.start_times[timer_name] = clock()
            return True
        return False

    def writeCameraToFile(self, cam_name):
        global cameraData, cameraSize
        width = 160
        height = 120
        if cameraSize[cam_name] == height*width:
            filename = STORAGE_PATH+cam_name+".ppm"
            with open(filename, "w") as f:
                f.write("P3\n")
                f.write(str(width) + " "+ str(height) + "\n")
                f.write("255\n")
                for y in range(height):
                    yy = height-1-y
                    for x in range(width):
                        # Each pixel has three components Red Green and Blue
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4]*255))+ " ")
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4+1]*255))+ " ")
                        f.write(str(int(cameraData[cam_name][yy*width*4+x*4+2]*255))+ " ")
                    f.write("\n")
        #logMessage("Camera Image saved at: "+str(filename))

    def acquire_images(self, write_to_file=True):        
        for camera_name in CAMERA_NAMES:
            requestCameraSensor(camera_name)
            if write_to_file and self.elapsed_time(camera_name, 5):
                self.writeCameraToFile(camera_name)


