import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load plane and humanoid
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(
    "/urdf/link-assem-9/urdf/link-assem-9.urdf",
    [0, 0, 0.4]
)

# === Friction tuning ===
p.changeDynamics(planeId, -1, lateralFriction=0.99, spinningFriction=0.05, rollingFriction=0.02)

num_joints = p.getNumJoints(robotId)
for j in range(num_joints):
    if "foot" in p.getJointInfo(robotId, j)[1].decode("utf-8"):
        p.changeDynamics(robotId, j, lateralFriction=0.95, spinningFriction=0.05, rollingFriction=0.02)
    else:
        p.changeDynamics(robotId, j, lateralFriction=0.5, spinningFriction=0.01, rollingFriction=0.01)

# Get joint dictionary
joint_dict = {}
print("==== Joint Info ====")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    name = info[1].decode('utf-8')
    joint_dict[name] = i
    print(i, name)

# Leg + hip joints
right_leg = [
    joint_dict["r-hip-roll"],
    joint_dict["r-top-thigh-pitch"],
    joint_dict["r-bottom-thigh-yaw"],
    joint_dict["r-calf-pitch"],
    joint_dict["r-foot-pitch"]
]

left_leg = [
    joint_dict["l-hip-roll"],
    joint_dict["l-top-thigh-pitch"],
    joint_dict["l-bottom-thigh-yaw"],
    joint_dict["l-calf-pitch"],
    joint_dict["l-foot-pitch"]
]

# Add hip pitch joints (these move pelvis forward/backward)
hip_joints = [
    joint_dict["r-hip-roll"],
    joint_dict["l-hip-roll"]
]

# Helper
def set_leg_positions(leg, positions, force=60, max_vel=4.0):
    for j, pos in zip(leg, positions):
        p.setJointMotorControl2(
            robotId, j,
            p.POSITION_CONTROL,
            targetPosition=pos,
            force=force,
            maxVelocity=max_vel
        )

def set_hip_motion(right_hip_angle, left_hip_angle, force=100, max_vel=3.0):
    p.setJointMotorControl2(robotId, joint_dict["r-top-thigh-pitch"],
                            p.POSITION_CONTROL, targetPosition=right_hip_angle,
                            force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robotId, joint_dict["l-top-thigh-pitch"],
                            p.POSITION_CONTROL, targetPosition=left_hip_angle,
                            force=force, maxVelocity=max_vel)

# Walking sequence (legs + pelvis sway forward)
gait = [
    # stance (hips slightly forward)
    ([0, 0.05, 0, 0.08, -0.04], [0, 0.05, 0, 0.08, -0.04], (0.05, 0.05)),
    # lift right leg, hips tilt forward
    ([0, -0.05, 0, 0.15, -0.04], [0, 0.05, 0, 0.08, 0], (0.1, 0.05)),
    # place right leg forward, hips shift forward
    ([0, 0.10, 0, -0.04, 0.05], [0, -0.04, 0, 0.08, -0.04], (0.05, 0.1)),
    # return stance
    ([0, 0.05, 0, 0.08, -0.04], [0, 0.05, 0, 0.08, -0.04], (0.05, 0.05)),
    # lift left leg, hips tilt forward
    ([0, 0.05, 0, 0.08, 0], [0, -0.05, 0, 0.15, -0.04], (0.05, 0.1)),
    # place left leg forward
    ([0, -0.04, 0, 0.08, -0.04], [0, 0.10, 0, -0.04, 0.05], (0.1, 0.05)),
    # stance
    ([0, 0.05, 0, 0.08, -0.04], [0, 0.05, 0, 0.08, -0.04], (0.05, 0.05))
]

# Run walking cycle
for step in range(6):
    for right_cmd, left_cmd, (hip_r, hip_l) in gait:
        set_leg_positions(right_leg, right_cmd)
        set_leg_positions(left_leg, left_cmd)
        set_hip_motion(hip_r, hip_l)
        for _ in range(240):  # ~1 sec
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

p.disconnect()
