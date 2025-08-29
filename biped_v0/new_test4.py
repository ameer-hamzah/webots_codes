import pybullet as p
import pybullet_data
import time
import numpy as np

# ========================
# 1. Setup PyBullet world
# ========================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load ground and humanoid
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(
    "/urdf/link-assem-9/urdf/link-assem-9.urdf",
    [0, 0, 0.4]
)

# Friction tuning (important for walking)
p.changeDynamics(planeId, -1, lateralFriction=1.0, spinningFriction=0.05, rollingFriction=0.02)

num_joints = p.getNumJoints(robotId)
joint_dict = {}
print("==== Joint Info ====")
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    name = info[1].decode("utf-8")
    joint_dict[name] = i
    print(i, name)

# ========================
# 2. Define leg joints
# ========================
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

# ========================
# 3. Helpers
# ========================
def set_leg_positions(leg, positions, force=60, max_vel=6.0):
    for j, pos in zip(leg, positions):
        p.setJointMotorControl2(
            robotId, j,
            p.POSITION_CONTROL,
            targetPosition=pos,
            force=force,
            maxVelocity=max_vel
        )

def set_hip_motion(r_hip_angle, l_hip_angle, force=100, max_vel=6.0):
    p.setJointMotorControl2(robotId, joint_dict["r-top-thigh-pitch"],
                            p.POSITION_CONTROL, targetPosition=r_hip_angle,
                            force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robotId, joint_dict["l-top-thigh-pitch"],
                            p.POSITION_CONTROL, targetPosition=l_hip_angle,
                            force=force, maxVelocity=max_vel)

# ========================
# 4. Walking parameters
# ========================
step_time = 0.8          # seconds per step
step_height = 0.18       # how high the leg lifts
step_length = 0.25       # forward stride length (bigger = more forward motion)
hip_sway = 0.07          # pelvis sway
lean_offset = 0.15       # constant forward lean to shift CoM
freq = 1.0 / step_time   # Hz

# ========================
# 5. Main walking loop
# ========================
t = 0.0
dt = 1.0 / 240.0
sim_time = 12.0  # seconds

# while t < sim_time:
while True:
    phase = 2 * np.pi * freq * t

    # Right leg (phase)
    r_hip_pitch = hip_sway * np.sin(phase)
    r_thigh_pitch = step_length * np.sin(phase) + lean_offset
    r_calf_pitch = -step_height * max(0, np.sin(phase))  
    right_cmd = [0, r_thigh_pitch, 0, r_calf_pitch, 0]

    # Left leg (opposite phase)
    l_hip_pitch = hip_sway * np.sin(phase + np.pi)
    l_thigh_pitch = step_length * np.sin(phase + np.pi) + lean_offset
    l_calf_pitch = -step_height * max(0, np.sin(phase + np.pi))
    left_cmd = [0, l_thigh_pitch, 0, l_calf_pitch, 0]

    # Apply
    set_leg_positions(right_leg, right_cmd)
    set_leg_positions(left_leg, left_cmd)
    set_hip_motion(r_hip_pitch, l_hip_pitch)

    # Step physics
    p.stepSimulation()
    time.sleep(dt)
    t += dt

p.disconnect()
