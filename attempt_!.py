import pybullet as p
import pybullet_data
import numpy as np
import time

# Setup PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.loadURDF("plane.urdf", [0, 0, 0])

# Load Robot
init_pos = [0.0, 0.0, 0.0, -2 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4, 0.0, 0.0, 0.05, 0.05]
robot = p.loadURDF("franka_panda/panda.urdf", [0,0,0], useFixedBase=True)
p.resetBasePositionAndOrientation(robot, [0, 0, 0], [0, 0, 0, 1])

# Number of controllable joints
num_joints = p.getNumJoints(robot)

# Target position and orientation for the end effector
target_pos = [0.5, 0, 0.5]  # example position
target_ori = p.getQuaternionFromEuler([0, 0, np.pi/2])  # example orientation

# Joint indices for the actuated joints
joint_indices = list(range(7)) + [11] # ignoring last 4 joints, as they are not actuated

# Set control mode to position control
p.setJointMotorControlArray(bodyUniqueId=robot,
                            jointIndices=joint_indices,
                            controlMode=p.POSITION_CONTROL,
                            forces=[500] * len(joint_indices))

# Get the joint info
joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]

# Loop until the user closes the program
while True:
    # Get the current position and orientation of the end effector
    state = p.getLinkState(robot, joint_infos[-1][0])
    ee_pos, ee_ori = state[0], state[1]

    # If the end effector is not at the target position, calculate the joint angles to move it there
    if np.linalg.norm(np.array(target_pos) - np.array(ee_pos)) > 0.01:
        joint_poses = p.calculateInverseKinematics(robot, joint_infos[-1][0], target_pos, target_ori)
        p.setJointMotorControlArray(bodyUniqueId=robot,
                                    jointIndices=joint_indices,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=joint_poses[:len(joint_indices)])

    # Step simulation
    p.stepSimulation()

    # Add some delay to match real-time
    time.sleep(0.01)
