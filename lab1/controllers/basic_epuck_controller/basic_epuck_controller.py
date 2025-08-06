"""basic_epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

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
    
    print(val)
    
    motor_left.setVelocity(values[7]/1000.0)
    motor_right.setVelocity(values[0]/1000.0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
