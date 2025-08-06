from controller import Robot, Motor

TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
# leftMotor.setPosition(float('inf'))
# rightMotor.setPosition(float('inf'))

# set up the motor speeds at 10% of the MAX_SPEED.
motor_left.setPosition(6.28)
motor_right.setPosition(0)
motor_left.setPosition(6.28)
motor_right.setPosition(6.28)

while robot.step(TIME_STEP) != -1:
   pass
