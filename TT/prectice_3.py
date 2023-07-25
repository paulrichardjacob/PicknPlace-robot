# Import necessary libraries
import numpy as np
import pybullet as p
import pybullet_data
import time
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

def generate_waypoints(current_position, target_position, num_waypoints):
    waypoints = np.linspace(current_position, target_position, num_waypoints)
    return waypoints

# Function to move the end effector to a target position
def move_to_target(target_position):
    # Get current state of robot
    joint_positions, end_effector_position, end_effector_quaternion = get_robot_state()
    joint_positions = np.array(joint_positions)

    # Calculate the joint positions needed to move end effector to the target position, keeping the same orientation
    target_joint_positions = p.calculateInverseKinematics(panda, 11, target_position, end_effector_quaternion)
    target_joint_positions = np.array(target_joint_positions[:9])  # Ignore last 2 values for gripper

    # Calculate the difference between the current and target joint positions
    diff_joint_positions = target_joint_positions - joint_positions

    # Set the joint motors to move towards the target joint positions
    p.setJointMotorControlArray(panda, range(9), p.VELOCITY_CONTROL, targetVelocities=diff_joint_positions)
    p.stepSimulation()  # Update the simulation

# Function to control the gripper
def control_gripper(wait_time=2):
    # Open the gripper
    target_positions = [0.04, 0.04]  # Open position
    p.setJointMotorControlArray(panda, [9, 10], p.POSITION_CONTROL, targetPositions=target_positions)
    p.stepSimulation()  # Update the simulation
    time.sleep(wait_time)  # Wait after opening the gripper
    # Close the gripper
    target_positions = [0, 0]  # Close position
    p.setJointMotorControlArray(panda, [9, 10], p.POSITION_CONTROL, targetPositions=target_positions)
    p.stepSimulation()  # Update the simulation

# Function to generate waypoints
def generate_waypoints(current_position, target_position, num_waypoints):
    waypoints = np.linspace(current_position, target_position, 10)
    return waypoints

# Define the desired position for the end effector
desired_position = np.array([0.3, 0.3, 0.5])

# Generate waypoints
_, current_position, _ = get_robot_state()
waypoints = generate_waypoints(current_position, desired_position, 10)  # Generate 10 waypoints

# Move end effector through each waypoint
for waypoint in waypoints:
    # Continue moving end effector towards waypoint until it is within a certain distance
    while np.linalg.norm(waypoint - np.array(get_robot_state()[1])) > 0.01:
        move_to_target(waypoint)
        time.sleep(1./240.)  # Sleep to make sure simulation runs at correct speed

# Grip the object
control_gripper(wait_time=2)

# Release the object
control_gripper(wait_time=2)

# Disconnect from Pybullet
p.disconnect()
