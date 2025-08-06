"""basic_epuck_controller controller."""

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
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

ds = []
for i in range(8):
    ds.append(robot.getDevice('ps'+str(i)))
    ds[-1].enable(timestep)
    
ls = []
for i in range(8):
    ls.append(robot.getDevice('ls'+str(i)))
    ls[-1].enable(timestep)
    

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    val = []
    for light in ls:
        val.append(light.getValue())
    val = np.asarray(val)
    val = val/4000*3.14

    d = []
    for dist in ds:
        d.append(dist.getValue())
    d=np.asarray(d)
    d=d/1000*3.14
    
    # print(val[7], val[0])
    print(d)
    
    phil=3.14-d[0]-d[1]-d[2]# +val[7]
    phir=3.14-d[7]-d[6]-d[5]# +val[0]
    
    motor_left.setVelocity(phil)
    motor_right.setVelocity(phir)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
