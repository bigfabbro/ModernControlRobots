from mars_interface import *
import random
import math

random.seed()

right_actuator = 0.
left_actuator = 0.
right = 0
left = 0


def randomWalk(distance):
    """A behavior that makes the robot randomly roam the area."""
    global right_actuator
    global left_actuator
    global right
    global left

    if distance[7] < 0.3 and distance[5] < 0.3:
        right_actuator = -10.;
    elif right < 0:
        right_actuator = random.random()*10.
        right = 10
    else:
        right -= 1

    if distance[2] < 0.3 or distance[0] < 0.3:
        left_actuator = -12.;
    elif left < 0:
        left_actuator = random.random()*10.
        left = 10
    else:
        left -= 1


def printValues(sll, slr):

    logMessage("actuator right value: %s" % slr)
    logMessage("actuator left value: %s" %sll)


def loveWalk(light, distance):

    global right_actuator
    global left_actuator

    # 1- approach the light --> if the sensors detect the light turn in its direction else walk at random
    # 2- during the robot is moving to the light decrease its speed
    # 3- when the robot is closer to the light stop it

    # Note: the speed decrease is calculated by this formula:
    #                                              v' = v - v * ((rs + ls)/ lv)
    # where:
    # - v': is the current speed
    # - v : is the default speed
    # - rs/ls : are respectvely the value of right sensor and the left sensor
    # - lv : is the limit value
    # It's easy to see that more the robot is closer to light source and more the quantity (rs+ls)/lv is closer to 1
    # and so the speed of the robot decrease

    # The eight light sensors are fused in two sensors. Each of the eight sensors has a different weight depending on
    # its position: the two external sensors have a eigher weight etc.
    sensor_light_left = 2 *light[0] + 1.5 * light[1] + 1.25 * light[2] + light[3]
    sensor_light_right = light[4] + 1.25 * light[5] + 1.5 * light[6] + 2 * light[7]

    # It's fixed a default speed for the two motors
    default_speed = 10.
    # This value is equal to the sum of the two sensors when the robot is sufficiently closer to the light source.
    limit_value = 3.3
    # if no light is detected the robot walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        randomWalk(distance)
    # if the difference between the light detected by the two sensors is lower than a fixed quantity, the robot can
    # walk straight
    elif abs(sensor_light_left-sensor_light_right)<0.1:
        left_actuator = default_speed - default_speed * ((sensor_light_right+sensor_light_left)/limit_value)
        right_actuator = default_speed - default_speed * ((sensor_light_right+sensor_light_left)/limit_value)
    # if the light detected by the left sensor is higher than the right one, the robot must turn on the left
    elif sensor_light_left > sensor_light_right:
        left_actuator = default_speed - default_speed * ((sensor_light_right+sensor_light_left)/limit_value)
    # if the light detected by the right sensor is higher than the left one, the robot must turn on the right
    elif sensor_light_right > sensor_light_left:
        right_actuator = default_speed - default_speed * ((sensor_light_right + sensor_light_left) / limit_value)


def aggressionWalk(light, distance):

    global right_actuator
    global left_actuator

    # The eight light sensors are fused in two sensors. Each of the eight sensors has a different weight depending on
    # its position: the two external sensors have a eigher weight etc.
    sensor_light_left = 2 *light[0] + 1.5 * light[1] + 1.25 * light[2] + light[3]
    sensor_light_right = light[4] + 1.25 * light[5] + 1.5 * light[6] + 2 * light[7]

    # Max speed of the motors
    max_speed = 12.

    # Value of the actuator in the same direction of the turn
    decr_speed = 7.

    # This value is equal at the value of the external sensor when the light is very closer. It's used for decrease
    # more the value of the actuator in same direction of the turn when the light is very closer
    limit_value = 1.8

    # if no light is detected the robot walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        randomWalk(distance)
    # if the difference between the light detected by the two sensors is lower than a fixed quantity, the robot can
    # walk straight at the maximum speed
    elif abs(sensor_light_right - sensor_light_left) <0.3:
        left_actuator = max_speed
        right_actuator = max_speed
    # if the value of the right sensor is higher than the one on the left, the robot must turn on the right with the
    # maximum speed. The value of the left actuator depends on how much the light is near the robot.
    elif sensor_light_right > sensor_light_left:
        left_actuator = max_speed
        right_actuator = decr_speed - decr_speed*(sensor_light_right/limit_value)
    # if the value of the left sensor is higher than the one on the right, the robot must turn on the left with the
    # maximum speed. The value of the right actuator depends on how much the light is near the robot.
    elif sensor_light_left > sensor_light_right:
        right_actuator = max_speed
        left_actuator = decr_speed - decr_speed*(sensor_light_left/limit_value)


def debugWalk(light, distance):

    global right_actuator
    global left_actuator

    sensor_light_left = 2 *light[0] + 1.5 * light[1] + 1.25 * light[2] + light[3]
    sensor_light_right = light[4] + 1.25 * light[5] + 1.5 * light[6] + 2 * light[7]

    right_actuator = 10.
    left_actuator = 9.

    logMessage("Right: %s" % sensor_light_right)
    logMessage("Left: %s" % sensor_light_left)

    












def doBehavior(distance, light, marsData):
    """Perform behavior depending on the "Robotik2/behavior" variable in
       MARS' "Configuration" menu."""
    global right_actuator, left_actuator

    behavior = marsData["Config"]["Robotik2"]["behavior"]
    if behavior > 0:
        if behavior == 1:
            loveWalk(light, distance)
        if behavior == 2:
            aggressionWalk(light, distance)
        if behavior == 3:
            debugWalk(light,distance)

    randomWalk(distance)

    # if timing(1):
    #     message = "sensor:"
    #     for s in light:
    #         message += " " + str(s)
    #     logMessage(message)
