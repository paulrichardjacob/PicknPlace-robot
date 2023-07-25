import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane and the robot
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
pandaEndEffectorIndex = 11

# Load the box to be picked up
boxId = p.loadURDF("cube_small.urdf", [0.5, 0, 0.2])

# Set the gravity
p.setGravity(0, 0, -9.81)

# Get the number of joints and links
numJoints = p.getNumJoints(panda)

# Define the joint indices
joint_indices = list(range(numJoints))

# The position and orientation of the end effector to grab the box
end_effector_pos = [0.5, 0, 0.2]
end_effector_orn = p.getQuaternionFromEuler([0, np.pi, 0])

# Use inverse kinematics to get the joint angles necessary to move the end effector to the box
jointPoses = p.calculateInverseKinematics(panda, pandaEndEffectorIndex, end_effector_pos, end_effector_orn)

# Move the robot to the position
for i, value in enumerate(jointPoses):
    p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, targetPosition=value)

# Wait for the motion to complete
time.sleep(2)

# Close the gripper to grab the box
p.setJointMotorControl2(panda, 9, p.POSITION_CONTROL, targetPosition=0, force=20)
p.setJointMotorControl2(panda, 10, p.POSITION_CONTROL, targetPosition=0, force=20)

# Wait for the gripper to close
time.sleep(2)

# Move the robot to a new position with the box
end_effector_pos = [0.5, 0, 0.5]
jointPoses = p.calculateInverseKinematics(panda, pandaEndEffectorIndex, end_effector_pos, end_effector_orn)

# Move the robot to the position
for i, value in enumerate(jointPoses):
    p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, targetPosition=value)

# Wait for the motion to complete
time.sleep(0.2)

while True:
    p.stepSimulation()
