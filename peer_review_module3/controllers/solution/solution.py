from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Setting values
maxVel = 6.28
state = 'FORWARD'

rotate_counter = 0       # counter for rotation duration
turn_count = 0           # number of turns made

# rotation durations (tuned experimentally)
rotate_steps_180 = int(1.55 * 1000 / timestep)   # ~180 degrees
rotate_steps_90  = int(0.78 * 1000 / timestep)   # ~90 degrees

# Getting motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

# Setting position control to infinity
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Getting distance sensors
distSensors= []
for i in range(8):
    distSensors.append(robot.getDevice('ps'+str(i)))
    distSensors[-1].enable(timestep)

# Main loop:
while robot.step(timestep) != -1:
    
    # ---- Read the sensors:
    ds = [dist.getValue() for dist in distSensors]

    # ---- Process state machine
    if state == 'FORWARD':
        left_wheel_vel = maxVel/2
        right_wheel_vel = maxVel/2

        # Stop condition ONLY after second turn is completed
        if turn_count >= 2 and ds[5] < 80.0:
            state = 'STOP'

        # If obstacle ahead, decide rotation direction
        elif ds[0] > 80.0 or ds[7] > 80.0:
            rotate_counter = 0
            if turn_count == 0:   # first obstacle → rotate left 180
                state = 'ROTATE_LEFT'
            elif turn_count == 1: # second obstacle → rotate right 90
                state = 'ROTATE_RIGHT'

    elif state == 'ROTATE_LEFT':
        left_wheel_vel = -maxVel/2
        right_wheel_vel = maxVel/2
        rotate_counter += 1

        # After ~180° turn, go back to forward
        if rotate_counter >= rotate_steps_180:
            state = 'FORWARD'
            turn_count += 1

    elif state == 'ROTATE_RIGHT':
        left_wheel_vel = maxVel/2
        right_wheel_vel = -maxVel/2
        rotate_counter += 1

        # After ~90° turn, go back to forward
        if rotate_counter >= rotate_steps_90:
            state = 'FORWARD'
            turn_count += 1

    elif state == 'STOP':
        left_wheel_vel = 0
        right_wheel_vel = 0

    # ---- Set motor velocities
    leftMotor.setVelocity(left_wheel_vel)
    rightMotor.setVelocity(right_wheel_vel)
