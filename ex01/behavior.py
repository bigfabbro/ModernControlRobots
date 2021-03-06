from mars_interface import *
import random
import math

random.seed()

right_actuator = 0.
left_actuator = 0.
right = 0
left = 0
fear = 0


def fuseSensors(light):

    # The eight light sensors are fused in two sensors. Each of the eight sensors has a different weight depending on
    # its position: the two external sensors have a higher weight etc.
    sensor_light_left = 2 * light[0] + 1.5 * light[1] + 1.25 * light[2] + light[3]
    sensor_light_right = light[4] + 1.25 * light[5] + 1.5 * light[6] + 2 * light[7]

    return sensor_light_right,sensor_light_left


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


def loveWalk(light, distance):

    global right_actuator
    global left_actuator

    # 1- approach the light --> if the sensors detect the light turn in its direction else walk at random
    # 2- during the robot is moving to the light decrease its speed
    # 3- when the robot is closer to the light stop it

    # Note: the speed decrease is calculated by this formula:
    #                                              v' = v * (1-((rs + ls)/ lv))
    # where:
    # - v': is the current speed
    # - v : is the default speed
    # - rs/ls : are respectively the value of right sensor and the left sensor
    # - lv : is the limit value
    # It's easy to see that more the robot is closer to light source and more the quantity (rs+ls)/lv is closer to 1
    # and so the speed of the robot decrease

    sensor_light_right, sensor_light_left = fuseSensors(light)

    # It's fixed a default speed for the two motors
    default_speed = 10.
    # This value is equal to the sum of the two sensors when the robot is sufficiently closer to the light source.
    # More is this limit value and more closer the robot goes to the light.
    limit_value = 3.3
    # It represents the difference between the two sensors. It is used to understand when the light is right in front 
    # of the robot and so it can go straight (slowing its velocity)
    precision = 0.1
    # if no light is detected the robot walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        randomWalk(distance)
    # if the difference between the light detected by the two sensors is lower than a fixed quantity, the robot can
    # walk straight, decreasing it velocity. The velocity is decreased accordingly to the distance from the light
    # (eg the intensity of the sensors), such that more is the intensity of the sensors and more is the decrease
    # of the velocity.
    # NOTE: the situation of having the two sensors with "the same" value could happen in two situation:
    # 1 when they have just caught some light, so that one sensor is low and the other is 0 (they are still "similar")
    # 2 when the light is straight in front of the robot
    elif abs(sensor_light_left-sensor_light_right)<precision:
        left_actuator = default_speed * (1-((sensor_light_right+sensor_light_left)/limit_value))
        right_actuator = default_speed * (1-((sensor_light_right+sensor_light_left)/limit_value))
    # if the light detected by the left sensor is higher than the right one, the robot must turn on the left
    # in order to do it, the velocity of the left motor is decreased by a value accordingly to the distance from the
    # light (as before)
    elif sensor_light_left > sensor_light_right:
        left_actuator = default_speed * (1-((sensor_light_right+sensor_light_left)/limit_value))
    # if the light detected by the right sensor is higher than the left one, the robot must turn on the right.
    # in order to do it, the velocity of the right motor is decreased by a value accordingly to the distance from the
    # light (as before)
    elif sensor_light_right > sensor_light_left:
        right_actuator = default_speed * (1-((sensor_light_right + sensor_light_left) / limit_value))


def aggressionWalk(light, distance):

    global right_actuator
    global left_actuator

    sensor_light_right, sensor_light_left = fuseSensors(light)

    default_speed = 10.

    # Value of the actuator in the same direction of the turn
    turn_speed = 7.

    # This value is equal to the value of the external sensor when the light is very close. It's used to decrease more
    # the value of the actuator in same direction of the turn and to encrease the velocity of the actuator in other side
    limit_value = 1.8

    # It represents the difference between the two sensors. It is used to understand when the light is right in front 
    # of the robot and so it can go straight (increasing its velocity)
    precision = 0.3
    # if no light is detected the robot walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        randomWalk(distance)
    # if the difference between the light detected by the two sensors is lower than a fixed quantity, the robot has to
    # walk straight and to accelerate, since it has the light's source in front of.
    elif abs(sensor_light_right - sensor_light_left) <precision:
        left_actuator = default_speed + default_speed * sensor_light_left/limit_value
        right_actuator = default_speed + default_speed * sensor_light_right/limit_value
    # if the value of the right sensor is higher than the one on the left, the robot have to turn on the right.
    # The value of the right actuator depends on how much the light is near to the robot, nearer is the
    # robot to the light's source and lower it is the velocity. In the other way, but with the same logic, the
    # velocity of the left actuator is encreased by a value that depends on how much the robot is closer to the
    # light
    elif sensor_light_right > sensor_light_left:
        left_actuator = default_speed + default_speed * sensor_light_right/limit_value
        right_actuator = turn_speed - turn_speed * sensor_light_right/limit_value
    # if the value of the left sensor is higher than the one on the right, the robot must turn on the left.
    # The value of the right actuator depends on how much the light is near the robot.
    elif sensor_light_left > sensor_light_right:
        right_actuator = default_speed + default_speed * sensor_light_left/limit_value
        left_actuator = turn_speed - turn_speed * sensor_light_left/limit_value

def fearWalk(light, distance):

    global right_actuator
    global left_actuator

    # This variable is used to prolong the escape of the robot from the light
    global fear

    sensor_light_right, sensor_light_left = fuseSensors(light)

    # Max speed of the motors
    max_speed = 12.

    # Value of the actuator in the same direction of the turn
    turn_speed = 5.

    # Duration in tic of the escape from the light since the value of the two sensors back to zero
    escape_tic = 10

    # This value is equal to the value of the external sensor when the light is very close. It's used for decrease
    # more the value of the actuator in same direction of the turn when the light is very closer
    limit_value = 1.8

    # If the two sensors have a value equal to zero, we can have two cases:
    # 1) the robot is escaping from the light, so it must escape until the counter "fear" isn't equal to zero or
    #    until it isn't closer to an obstacle
    # 2) the robot is away from the light, so it can walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        if fear > 0 and distance[7] > 0.3 and distance[5] > 0.3 and distance[2] > 0.3 and distance[0] > 0.3:
            right_actuator = max_speed
            left_actuator = max_speed
            fear -= 1
        else:
            fear = 0
            randomWalk(distance)

    # If the left sensor has a higher value than the right one, it must turn on the right with the maximum speed. So the
    # left actuator is placed at max_speed and the right one is placed at turn_speed ( << max_speed) minus a quantity
    # dependent on the nearness of the light ( in this way the robot can turn more quickly if the light is closer).
    elif sensor_light_left > sensor_light_right:
        left_actuator = max_speed
        right_actuator = turn_speed * (1 - sensor_light_left/limit_value)
        fear = escape_tic

    # If the right sensor has a higher value than the left one, it must turn on the left with the maximum speed. So the
    # right actuator is placed at max_speed and the left one is placed at turn_speed ( << max_speed) minus a quantity
    # dependent on the nearness of the light ( in this way the robot can turn more quickly if the light is closer).
    elif sensor_light_right > sensor_light_left:
        right_actuator = max_speed
        left_actuator = turn_speed * (1 - sensor_light_right/limit_value)
        fear = escape_tic


def curiosityWalk(light, distance):
    global right_actuator
    global left_actuator


    default_speed = 10.
    # This value is equal to the sum of the two sensors when the robot is sufficiently closer to the light source.
    # More is this limit value and more closer the robot goes to the light.
    limit_value = 2.6
    # It represents the difference between the two sensors. It is used to understand when the light is right in front 
    # of the robot and so it can go straight (slowing its velocity)
    precision = 0.3

    sensor_light_right, sensor_light_left = fuseSensors(light)

    # If no light is detected the robot walk at random
    if sensor_light_right == 0 and sensor_light_left == 0:
        randomWalk(distance)
    # If the sum of the two sensor is higher than limit value, the robot can go away. We can have three cases:
    # 1) the light is on the right, the robot will turn on the left, so the right actuator must be faster than the left one
    # 2) the light is on the left, the robot will turn on the right, so the left actuator must be faster than the right one
    # 3) the light is front of the robot, it will go in the opposite direction.
    elif (sensor_light_left+sensor_light_right) >= limit_value:
        if sensor_light_left > sensor_light_right:
            left_actuator = default_speed
        elif sensor_light_right > sensor_light_left:
            right_actuator = default_speed
        elif sensor_light_left == sensor_light_right:
            right_actuator = default_speed
            left_actuator = -default_speed

    # if the difference between the light detected by the two sensors is lower than a fixed quantity, the robot have to
    # walk straight decreasing its velocity, since it has the light's source in front of.
    elif abs(sensor_light_left-sensor_light_right)<precision:
        left_actuator = default_speed * (1.2-((sensor_light_right+sensor_light_left)/limit_value))
        right_actuator = default_speed * (1.2-((sensor_light_right+sensor_light_left)/limit_value))
    # if the light detected by the left sensor is higher than the right one, the robot must turn on the left
    # in order to do it, the velocity of the left motor is decreased by a value accordingly to the distance from the
    # light
    elif sensor_light_left > sensor_light_right:
        left_actuator = default_speed * (1.2-((sensor_light_right+sensor_light_left)/limit_value))
    # if the light detected by the right sensor is higher than the left one, the robot must turn on the right
    # in order to do it, the velocity of the right motor is decreased by a value accordingly to the distance from the
    # light
    elif sensor_light_right > sensor_light_left:
        right_actuator = default_speed * (1.2-((sensor_light_right + sensor_light_left) / limit_value))

def tortoiseWalk(light,distance):

    global right_actuator
    global left_actuator

    # A light with intensity higher than it can be define as a strong light.
    strong_light = 3

    sensor_light_right, sensor_light_left = fuseSensors(light)


    # We can combine the behaviours of the Braitenberg's vehicles to implement the Grey Walter's tortoise.
    # If the two sensors are both equal to zero, the robot search for a weak light. If the robot detects a weak
    # light (sensor_light_left + sensor_light_right < strong_light) it moves to the weak light (= love walk). When
    # the robot perceives a strong light (sensor_light_right + sensor_light_left >= strong_light)it turns away from
    # the strong light (= fear walk).
    if sensor_light_left == 0 and sensor_light_right == 0:
        randomWalk(distance)
    elif (sensor_light_right+sensor_light_left) >= strong_light:
        fearWalk(light,distance)
    else:
        loveWalk(light,distance)

def doBehavior(distance, light, marsData):
    """Perform behavior depending on the "Robotik2/behavior" variable in
       MARS' "Configuration" menu."""
    global right_actuator, left_actuator

    behavior = marsData["Config"]["Robotik2"]["behavior"]
    if 0 < behavior < 6:
        if behavior == 1:
            loveWalk(light, distance)
        elif behavior == 2:
            aggressionWalk(light, distance)
        elif behavior == 3:
            fearWalk(light,distance)
        elif behavior ==4:
            curiosityWalk(light,distance)
        elif behavior == 5:
            tortoiseWalk(light,distance)
    else:
        randomWalk(distance)

    # if timing(1):
    #     message = "sensor:"
    #     for s in light:
    #         message += " " + str(s)
    #     logMessage(message)
