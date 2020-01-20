from mars_interface import *
from mars_plugin import *
import random
import intersect
import math
import numpy as np
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
- random_percentage: number of particles (in percentage) which have to be drawn randomly at each time step
- basePos: base positions of the lasers
- directions: directions of the lasers
- alphas: robot-specific error parameters, which specify the error accrued with motion (in this case randomly chosen)
- search: two possible values "global" and "local". If global the particles are positioned all around the map. 
          If local the particles are positioned in the environment around the wheelchair.
- angle: two possible values "random" and "fixed". If random the particles are generated with random orientation. 
        If fixed the particles are generated with an orientation chosen between fixed angles. 
"""
map_size_x = 8
map_size_y = 6
laser_max_range = 4.
update_time = 0.02
wheel_distance = 0.485751326
radius = 0.15
laser_offset = 0.349
random_percentage = 0.1
basePos = np.array([[0.320234, 0.230233],
                    [0.320234, -0.230234],
                    [-0.306395, 0.214315],
                    [-0.306395, -0.214316]])
directions = np.array([0.262, -1.309, 1.833, -2.879])
alphas = [0.1, 0.1, 1, 1]
search = "global"
angle = "fixed"

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

def getOdometryMeasurement(joystickLeft, joystickRight):
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


def motionModel(particle, u_t):
    global alphas
    # u_t is transformed into a sequence of three steps:
    # a first rotation (rot_1), a translation (trans) and a second rotation rot_2
    delta_rot_1 = math.atan2(u_t[1], u_t[0])
    delta_trans = math.sqrt((u_t[0]) ** 2 + (u_t[1]) ** 2)
    delta_rot_2 = (u_t[2]) - delta_rot_1

    # Here is assumed that these three parameters are affected by independent noise.
    # In particular the values of noises are drawn from a  zero centered normal distribution and with variance
    # dependent by vector of robot- specific alpha parameters
    rot_1 = delta_rot_1 - np.random.normal(0, abs(alphas[0] * delta_rot_1 + alphas[1] * delta_trans))
    trans = delta_trans - np.random.normal(0, abs(alphas[2] * delta_trans + alphas[3] * (delta_rot_1 + delta_rot_2)))
    rot_2 = delta_rot_2 - np.random.normal(0, abs(alphas[0] * delta_rot_2 + alphas[1] * delta_trans))

    new_particle = np.zeros(3)
    # Determining the new position and orientation of the particle
    new_particle[0] = particle[0] + trans * np.cos(particle[2]+ rot_1)
    new_particle[1] = particle[1] + trans * np.sin(particle[2]+ rot_1)
    new_particle[2] = particle[2] + rot_1 + rot_2
    return new_particle

def perceptionModel(particle):
    """

    :param particle: the particle of which we want to estimate the measurements
    :return: sse: sum of square error between the actual measurements and the estimated ones (weight of the particle)
    """
    global basePos, directions, laser_offset, laser_max_range
    sensor_pos = []
    laser_end_pos = []
    # x_sens and y_sens are the coordinates of the starting point of the i-th group of sensors in the current particle.
    # Note: Because of our basePos (laser base positions) are calculated from the center of the robot and our particle is
    # positioned in center of the wheels, in order to estimate the position of the laser in the current particle we
    # need to add a 0.255 (distance between the center of the wheels and the center of the wheelchair on the x-axis).
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
    sse = np.sum(np.square(distance-laser_values))

    """
    Used to verify that the lasers are correctly represented in each particle
    
    clearLines("path")
    configureLines("path", 5, 0.2, 0.8, 0.2)
    for i in range(len(sensor_pos)):
        for j in range(4):
            appendLines("path", sensor_pos[i][0], sensor_pos[i][1], 0.5)
            appendLines("path", laser_end_pos[4*i+j][0], laser_end_pos[4*i+j][1], 0.5)
    """
    return sse


def normalization(w, max):
    """
        :param w: vector of weights (shape: num of particles)
        :param max: max weight
        :return: w: normalized vector of weights. The sum of all the weights is 1 and the particle with maximum weight
                has less probability to be drawn.
    """
    for k in range(len(w)):
        w[k] = (max+1 - w[k])/(max+1)
    w_sum = np.sum(w)
    w = np.divide(w,w_sum)
    return  w

def particle_filter(particles, u_t, pos):
    """
    :param particles: vector of particles (shape: 3 x num of particles)
    :param u_t: control input (length: 3) in our case is the position estimated by odometry
    :param pos: current position of the robot (used for the first test of motion model)
    :return: x_t: set of particles at time t (shape: 3 x num of particles)
    """
    global map_size_x, map_size_y, random_percentage, search, angle
    max_weight = 0
    weights = []
    x_cap = []
    # First phase of particle filter
    for i in range(getNumberOfParticles()):
        # Motion model is applied to the particle to predict next state of the particle
        # Theoretically, we are sampling from the conditional probability <x_t | u_t, x_t-1>
        x_cap.append(motionModel(particles[i], u_t))
        # Perception model is applied to the particle to obtain its current weight
        # In this case
        weights.append(perceptionModel(particles[i]))
        """
        used to test the correctness of the motion model with the distance from the current position of the robot
        
        distance = dist(pos, particles[i])
        weights.append(distance)
        """
        if weights[i] > max_weight:
            max_weight = weights[i]
    prob = normalization(weights, max_weight)
    rand_particles = int(getNumberOfParticles() * random_percentage)
    indices = np.random.choice([int(i) for i in range(0, getNumberOfParticles())],
                               size=getNumberOfParticles() - rand_particles, replace=True, p=prob)
    x_t = (np.asarray(x_cap)[indices])
    for i in range(rand_particles):
        if search == "global":
            if angle == "fixed":
                x_t = np.append(x_t, [[random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y, map_size_y),
                                       random.choice([0, -np.pi, np.pi, np.pi/2, -np.pi/2])]], axis=0)
            else:
                x_t = np.append(x_t, [[random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y, map_size_y),
                                       random.uniform(0, 2*np.pi)]], axis=0)
        else:
            if angle == "fixed":
                x_t = np.append(x_t, [[pos[0]+random.uniform(-2,2), pos[1]+random.uniform(-2, 2),
                                       random.choice([0, np.pi, np.pi / 2, -np.pi / 2])]], axis=0)
            else:
                x_t = np.append(x_t, [[pos[0] + random.uniform(-2, 2), pos[1] + random.uniform(-2, 2),
                                       random.uniform(0, 2*np.pi)]], axis=0)
    return x_t


def doBehavior(distance, direction, pos, marsData, waypoints, walls, joystickLeft, joystickRight):
    global first_time, particles, map_size_x, map_size_y, search, angle

    """
    pos may only be used as a reference
    print "Real Position: " + str(pos[0]) + ":" + str(pos[1]) + ":" + str(pos[2])
    print "using %d particles" % getNumberOfParticles()
    """
    if first_time:
        pData = pointCloudData["particles"]
        for i in range(getNumberOfParticles()):
            if search == "global":
                if angle == "fixed":
                    particle = [random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y,map_size_y),
                                random.choice([0, np.pi, -np.pi, np.pi/2, -np.pi/2])]
                else:
                    particle = [random.uniform(-map_size_x, map_size_x), random.uniform(-map_size_y, map_size_y),
                                random.uniform(0, 2*np.pi)]
            else:
                if angle == "fixed":
                    particle = [pos[0]+random.uniform(-2, 2), pos[1]+random.uniform(-2, 2),
                                random.choice([0, np.pi, np.pi/2, -np.pi/2])]
                else:
                    particle = [pos[0] + random.uniform(-2, 2), pos[1] + random.uniform(-2, 2),
                                random.uniform(0, 2*np.pi)]
            particles.append(particle)
        first_time = False

    #print "Joystick input: (" + str(joystickLeft) + " : " +str(joystickRight) +")" + " direction: " + str(direction)
    odom = getOdometryMeasurement(joystickLeft, joystickRight)
    particles = particle_filter(particles, odom, pos)

    """
    print "Sensors:"
    always counterclockwise when looking from above
    print "fl (%.2f, %.2f, %.2f, %.2f)" % (distance[0], distance[1],distance[2],distance[3])
    print "fr (%.2f, %.2f, %.2f, %.2f)" % (distance[4], distance[5],distance[6],distance[7])
    print "bl (%.2f, %.2f, %.2f, %.2f)" % (distance[8], distance[9],distance[10],distance[11])
    print "br (%.2f, %.2f, %.2f, %.2f)" % (distance[12], distance[13],distance[13],distance[15])
    Laser center is 45 deg twisted in each case
    Angular offset between the laser beams: 0.349 rad
    """
    points = []
    for p in particles:
        points.append([p[0], p[1]])

    #An array of points can be returned, these are drawn in the 3D scene
    return points
