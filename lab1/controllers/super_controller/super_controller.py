"""super_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

max_vel = 3.14

# Getting motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')


# Setting position control to infinity
# So that it can be controlled via velocity commands

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Getting sensors in a python list
distSensors= []
for i in range(8):
    distSensors.append(robot.getDevice('ps'+str(i)))
    distSensors[-1].enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    val = []
    for dist in distSensors:
        val.append(dist.getValue())
    
    val = np.asarray(val)
    val = val/1000*3.14
    print(val)
    
    right_wheel_vel = max_vel-val[7]-val[6]-val[5]
    left_wheel_vel = max_vel-val[0]-val[1]-val[2]

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    leftMotor.setVelocity(left_wheel_vel)
    rightMotor.setVelocity(right_wheel_vel)
    # rightMotor.setVelocity(0)
    # leftMotor.setVelocity(0)

# Enter here exit cleanup code.
