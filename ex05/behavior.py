from mars_interface import *
from mars_plugin import *
import random
import intersect
import math
import numpy as np
import time
first_time = True


##########################################  CONFIGURATION PARAMETERS ##################################################
"""
- map_size_x: dimension of the map on the x-axis
- map_size_y: dimension of the map on the y-axis
- laser_max_range: range of the laser sensor
- update_time: time lapse between each call of the method (measured with time library)
- wheel_distance: distance between the wheels of the robot
- radius: wheel radius
- laser_offset: offset between the lasers (rad/s)
- basePos: base positions of the lasers
- directions: directions of the lasers
- alphas: robot-specific error parameters, which specify the error accrued with motion (in this case randomly chosen)
"""
map_size_x = 7
map_size_y = 6
laser_max_range = 4.
update_time = 0.02
wheel_distance = 0.485751326
radius = 0.15
laser_offset = 0.349
basePos = np.array([[0.320234, 0.230233],
                    [0.320234, -0.230234],
                    [-0.306395, 0.214315],
                    [-0.306395, -0.214316]])
directions = np.array([0.262, -1.309, 1.833, -2.879])
alphas = [0.2, 0.2, 1., 1.]

#######################################################################################################################

particles =[]

def checkIntersect(pos, wp):
    
    # intersect.get_intersect has a vector as parameter, defined by two position
    # returns 2 if no intersection with walls
    # returns value between 0 and 1 otherwise
    # The value is the percentage of overall the vector length where the first intersection occurred

    if intersect.get_intersect(pos[0], pos[1], wp[0], wp[1]) < 1:
       return False
    return True


def dist(p1, p2):
    return np.sum(np.square(np.subtract(p1[0:2], p2[0:2])))

#please use this function to set the number of particles
#(initializes the array to draw the particles)
def getNumberOfParticles():
    return 1000

def getOdometry(joystickLeft, joystickRight):
    global update_time, wheel_distance, radius
    # displacement of the left(dl)/right(dr) wheel due to the input of the left/right joystick.
    # d_l and d_r are calculated by the formula:
    #                               d=2*pi*R*nrs*t
    # where nrs is the number of revolution per second of the wheel (w/(2*pi)).

    d_l =  joystickLeft * radius * update_time
    d_r =  joystickRight * radius * update_time

    # displacement of the center of the axis between the wheels
    d_c = (d_l + d_r)/2
    # angular direction displacement of the center
    theta = (d_r - d_l)/wheel_distance
    #
    new_odometry = [d_c * np.cos(theta), d_c * np.sin(theta), theta]
    return new_odometry


def motionModel(particle, est_pos):
    global alphas

    delta_rot_1 = math.atan2(est_pos[1], est_pos[0])
    delta_trans = math.sqrt((est_pos[0])**2 + (est_pos[1])**2)
    delta_rot_2 =(est_pos[2]) - delta_rot_1

    rot_1 = delta_rot_1 - np.random.normal(0, abs(alphas[0] * delta_rot_1 + alphas[1] * delta_trans))
    trans = delta_trans - np.random.normal(0, abs(alphas[2] * delta_trans + alphas[3] * (delta_rot_1 + delta_rot_2)))
    rot_2 = delta_rot_2 - np.random.normal(0, abs(alphas[0] * delta_rot_2 + alphas[1] * delta_trans))

    new_particle = np.zeros(3)
    new_particle[0] = particle[0] + trans * np.cos(particle[2]+ rot_1)
    new_particle[1] = particle[1] + trans * np.sin(particle[2]+ rot_1)
    new_particle[2] = particle[2] + rot_1 + rot_2
    return new_particle

def perceptionModel(particle):
    global basePos, directions, laser_offset, laser_max_range
    sensor_pos = []
    laser_end_pos = []
    # x_sens and y_sens are the coordinates of the starting point of the i-th group of sensors in the current particle.
    for i in range(len(basePos)):
        x_sens = particle[0] + np.cos(particle[2]) * (basePos[i][0]+0.255) - np.sin(particle[2]) * basePos[i][1]
        y_sens = particle[1] + np.sin(particle[2]) * (basePos[i][0]+0.255) + np.cos(particle[2]) * basePos[i][1]
        sensor_pos.append([x_sens, y_sens])
        # x_end and y_end are the coordinates of the ending point of the j-th laser sensor of the i-th group
        for j in range(4):
            x_end = x_sens + laser_max_range * np.cos(particle[2] + directions[i] + j * laser_offset)
            y_end = y_sens + laser_max_range * np.sin(particle[2] + directions[i] + j * laser_offset)
            laser_end_pos.append([x_end, y_end])
    laser_values = []
    # Evaluating "intersect" of the lasers with the walls of the map in the current particle
    for i in range(len(sensor_pos)):
        for j in range(4):
            value = intersect.get_intersect(sensor_pos[i][0], sensor_pos[i][1], laser_end_pos[4*i+j][0], laser_end_pos[4*i+j][1])
            if value == 2:
                laser_values.append(laser_max_range)
            else:
                laser_values.append(value * laser_max_range)
    # Calculating the SSE
    quadratic_error = np.sum(np.square(distance-laser_values))

    """
    Used to verify that the lasers are correctly represented in each particle
    
    for i in range(len(sensor_pos)):
        for j in range(4):
            appendLines("path", sensor_pos[i][0], sensor_pos[i][1], 0.5)
            appendLines("path", laser_end_pos[4*i+j][0], laser_end_pos[4*i+j][1], 0.5)
    """
    return quadratic_error


def normalization(w, max):
    for k in range(len(w)):
        w[k] = (max+1 - w[k])/(max+1)
    w_sum = np.sum(w)
    w = np.divide(w,w_sum)
    return  w

def particle_filter(particles, u_t,pos):
    global map_size_x, map_size_y, entropy, entropy_sum, entropy_counter
    max_weight = 0
    weights = []
    x_cap = []
    current_entropy = 0
    x_t = []
    for i in range(getNumberOfParticles()):
        x_cap.append(motionModel(particles[i], u_t))
        weights.append(perceptionModel(particles[i]))
        current_entropy += weights[i]*np.log(weights[i])
        #distance = dist(pos, particles[i])
        #weights.append(distance)
        if weights[i] > max_weight:
            max_weight = weights[i]
    prob = normalization(weights, max_weight)
    """
    if current_entropy > entropy*1.2:
        alp1 = 0.3
    elif current_entropy > entropy*1.4:
        alp1 = 0.5
    else:
        alp1 = 0.
    """
    alp1 = 0.1
    rand_particles = int(getNumberOfParticles() * alp1)
    indeces = np.random.choice([int(i) for i in range(0, getNumberOfParticles())],
                               size=getNumberOfParticles() - rand_particles, replace=True, p=prob)
    x_t = (np.asarray(x_cap)[indeces])
    for i in range(int(getNumberOfParticles() * alp1)):
        x_t = np.append(x_t, [[random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y, map_size_y),
                               random.choice([0, -np.pi, np.pi, np.pi/2, -np.pi/2])]], axis = 0 )

    entropy = current_entropy
    #logMessage("entr: %s" %entropy)
    #new_particles_indices = np.random.choice([int(i) for i in range(0,getNumberOfParticles())], size=getNumberOfParticles(), replace=True, p=prob)
    #x_t = np.asarray(x_cap)[new_particles_indices]
    return x_t


def doBehavior(distance, direction, pos, marsData, waypoints, walls, joystickLeft, joystickRight):
    global first_time, particles, map_size_x, map_size_y, timebefore, update_time
    #pos may only be used as a reference
    print "Real Position: " + str(pos[0]) + ":" + str(pos[1]) + ":" + str(pos[2])

    print "using %d particles" % getNumberOfParticles()
    
    if first_time:
        pData = pointCloudData["particles"]
        for i in range(getNumberOfParticles()):
            #particle = [pos[0], pos[1], pos[2]]
            particle = [random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y,map_size_y), random.choice([0, np.pi, -np.pi, np.pi/2, -np.pi/2])]
            particles.append(particle)
            #logMessage("original particles %s" %particles)

        first_time = False
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)

    
    # Laser scanner positions fl, fr, bl, br


    
    print "Joystick input: (" + str(joystickLeft) + " : " +str(joystickRight) +")" + " direction: " + str(direction)
    new_odom = getOdometry(joystickLeft, joystickRight)
    particles = particle_filter(particles, new_odom, pos)
    #logMessage(" pos%s" %pos)
    #logMessage("particles %s" %particles)
    print "Sensors:"
    #always counterclockwise when looking from above
    print "fl (%.2f, %.2f, %.2f, %.2f)" % (distance[0], distance[1],distance[2],distance[3])
    print "fr (%.2f, %.2f, %.2f, %.2f)" % (distance[4], distance[5],distance[6],distance[7])
    print "bl (%.2f, %.2f, %.2f, %.2f)" % (distance[8], distance[9],distance[10],distance[11])
    print "br (%.2f, %.2f, %.2f, %.2f)" % (distance[12], distance[13],distance[13],distance[15])
    #Laser center is 45 deg twisted in each case
    #Angular offset between the laser beams: 0.349 rad
    points = []
    for p in particles:
        points.append([p[0], p[1]])
    #An array of points can be returned, these are drawn in the 3D scene
    return points
