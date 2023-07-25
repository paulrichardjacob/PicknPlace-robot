import pybullet as p
import time
import pybullet_data
import numpy as np
import pybullet_planning as pp
import burg_toolkit as burg
# Initialize Pybullet physics simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path for PyBullet data
p.setGravity(0, 0, -10)  # Set gravity in the simulation
p.loadURDF("plane.urdf", [0, 0, 0])  # Load ground plane

# Load Panda robot
panda = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Function to get current state of the robot
def get_robot_state():
    # Get joint states for all joints (0-8 for arm, 9-10 for gripper)
    joint_states = p.getJointStates(panda, range(9))
    joint_positions = [state[0] for state in joint_states]  # Extract joint positions from joint states
    end_effector_state = p.getLinkState(panda, 11)  # Get the state of the end effector (link 11)
    end_effector_position = end_effector_state[4]  # Extract end effector position
    end_effector_quaternion = end_effector_state[5]  # Extract end effector quaternion (orientation)
    return joint_positions, end_effector_position, end_effector_quaternion

def move_to_target(target_pose):
    # Get current state of robot
    joint_positions, end_effector_position, end_effector_quaternion = get_robot_state()
    joint_positions = np.array(joint_positions)

    # Convert target_pose to position and quaternion
    target_position, target_quaternion = burg.util.position_and_quaternion_from_tf(target_pose, convention='pybullet')

    # Calculate the joint positions needed to move end effector to the target position, keeping the same orientation
    target_joint_positions = p.calculateInverseKinematics(panda, 11, target_position, target_quaternion)
    target_joint_positions = np.array(target_joint_positions[:9])  # Ignore last 2 values for gripper

    # Calculate the difference between the current and target joint positions
    diff_joint_positions = target_joint_positions - joint_positions

    # Set the joint motors to move towards the target joint positions
    p.setJointMotorControlArray(panda, range(9), p.VELOCITY_CONTROL, targetVelocities=diff_joint_positions)
    p.stepSimulation()  # Update the simulation

# Define the desired orientation for the end effector
desired_pose = np.array([
    [-1.0, 0.0, 0.0, 0.2],
    [0.0, 1.0, 0.0, 0.3],
    [0.0, 0.0, -1.0, 0.5],
    [0.0, 0.0, 0.0, 1.0]
])

# Continue moving end effector towards box until it is within a certain distance
while np.linalg.norm(np.array(box_position) - np.array(get_robot_state()[1])) > 0.05:
    target_pose = desired_pose
    move_to_target(target_pose)
    time.sleep(1./240.)  # Sleep to make sure simulation runs at correct speed

# Move the box to the target location
target_pose = np.array([
    [1.0, 0.0, 0.0, -0.5],
    [0.0, 0.0, 1.0, 0.5],
    [0.0, -1.0, 0.0, 0.5],
    [0.0, 0.0, 0.0, 1.0]
])

# Continue moving end effector towards target location until it is within a certain distance
while np.linalg.norm(np.array(burg.util.position_and_quaternion_from_tf(target_pose, convention='pybullet')[0]) - np.array(get_robot_state()[1])) > 0.05:
    move_to_target(target_pose)
    # To check for collision
    for i in range(p.getNumJoints(panda)):
        collision = p.getClosestPoints(panda, planeId, 0, linkIndexA=i)
        if len(collision) > 0:
            print(f"Link index {i} is in collision")
    time.sleep(1./240.)  # Sleep to make sure simulation runs at correct speed

# Disconnect from Pybullet
p.disconnect()