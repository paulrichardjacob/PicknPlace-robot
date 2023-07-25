import pybullet as p
import time
import pybullet_data
import numpy as np
import path_planning as pp
import burg_toolkit as burg

# Initialize Pybullet physics simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path for PyBullet data
p.setGravity(0, 0, -10)  # Set gravity in the simulation
p.loadURDF("plane.urdf", [0, 0, 0])  # Load ground plane

# Load Panda robot
panda = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Define joint limits for the Franka Panda robot
joint_limits = [[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973]]

# Define collision checking function
def robot_collision_check(joint_configuration):
    # Set robot joints to given configuration
    p.setJointMotorControlArray(panda, list(range(7)), p.POSITION_CONTROL, joint_configuration)
    p.stepSimulation()  # Update the simulation

    # Perform collision check
    collisions = p.getContactPoints(panda)

    # Return False if any collisions are detected
    return len(collisions) == 0

# Define sample function
def robot_sample_fn():
    # Generate random configuration within joint limits
    return [np.random.uniform(low=low, high=high) for low, high in joint_limits]

# Define extend function
def robot_extend_fn(config1, config2):
    # Compute the difference between configurations
    diff = np.array(config2) - np.array(config1)

    # Normalize the difference
    diff /= np.linalg.norm(diff)

    # Generate the path
    path = [config1 + i * 0.01 * diff for i in range(100)]

    return path

# Function to get current state of the robot
def get_robot_state():
    # Get joint states for all joints (0-8 for arm, 9-10 for gripper)
    joint_states = p.getJointStates(panda, range(9))
    joint_positions = [state[0] for state in joint_states]  # Extract joint positions from joint states
    end_effector_state = p.getLinkState(panda, 11)  # Get the state of the end effector (link 11)
    end_effector_position = end_effector_state[4]  # Extract end effector position
    end_effector_quaternion = end_effector_state[5]  # Extract end effector quaternion (orientation)
    return joint_positions, end_effector_position, end_effector_quaternion

# Define the desired orientation for the end effector
desired_pose = np.array([
    [-1.0, 0.0, 0.0, 0.2],
    [0.0, 1.0, 0.0, 0.3],
    [0.0, 0.0, -1.0, 0.5],
    [0.0, 0.0, 0.0, 1.0]
])

# Define start and goal configurations for the robot
start = get_robot_state()[0]  # Current joint positions
goal = p.calculateInverseKinematics(panda, 11, burg.util.position_and_quaternion_from_tf(desired_pose, convention='pybullet')[0])[:7]  # Desired joint positions

# Call the PRM algorithm to generate a path
path = pp.prm(np.array(start), np.array(goal), np.linalg.norm, robot_sample_fn, robot_extend_fn, robot_collision_check, num_samples=200)

# Follow the path
for config in path:
    p.setJointMotorControlArray(panda, list(range(7)), p.POSITION_CONTROL, config)
    p.stepSimulation()  # Update the simulation
    time.sleep(1./240.)  # Sleep to make sure simulation runs at correct speed

# Disconnect from Pybullet
p.disconnect()
