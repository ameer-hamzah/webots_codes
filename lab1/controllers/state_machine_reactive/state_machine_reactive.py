"""super_controller controller."""

###############################################
####                Module 3               ####
#### Reactive Behaviors and State Machines ####
###############################################

from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Maximum velocity allowed (Max=6.28)
max_vel = 3.14
state = 'FOLLOW'

# Getting motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')


# Setting position control to infinity
# So that it can be controlled via velocity commands

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Getting distance sensors in a python list
distSensors= []
for i in range(8):
    distSensors.append(robot.getDevice('ps'+str(i)))
    distSensors[-1].enable(timestep)

# Getting light sensors in a python list
lightSensors = []
for i in range(8):
    lightSensors.append(robot.getDevice('ls'+str(i)))
    lightSensors[-1].enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # Read the distance sensor
    dist_val = []
    for dist in distSensors:
        dist_val.append(dist.getValue())
    
    #
    dist_val = np.asarray(dist_val)
    dist_val = dist_val/1000*3.14
    print(dist_val)
    
    light_val = []
    for light in lightSensors:
        light_val.append(light.getValue())
    
    light_val = np.asarray(light_val)
    light_val = light_val/4000*3.14
    print(light_val)
    
    
    if state == 'FOLLOW':
        right_wheel_vel = max_vel-dist_val[7]-dist_val[6]-dist_val[5]+light_val[0]+light_val[1]
        left_wheel_vel = max_vel-dist_val[0]-dist_val[1]-dist_val[2]+light_val[7]+light_val[6]
        if(max(dist_val)>1):
            state = 'AVOID'
    elif state == 'AVOID':
        right_wheel_vel = max_vel-dist_val[7]-dist_val[6]-dist_val[5]
        left_wheel_vel = max_vel-dist_val[0]-dist_val[1]-dist_val[2]
        if(max(dist_val)<1):
            state = 'FOLLOW'

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    leftMotor.setVelocity(left_wheel_vel)
    rightMotor.setVelocity(right_wheel_vel)
    # rightMotor.setVelocity(0)
    # leftMotor.setVelocity(0)

# Enter here exit cleanup code.
