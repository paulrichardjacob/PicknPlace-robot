import pybullet as p
import pybullet_data
import pybullet_planning as pp
import numpy as np
import burg_toolkit as burg
import time
from pybullet_planning.interfaces.robots.collision import get_collision_fn


def pose_to_joint_conf(robot, end_effector_link, desired_pose):
    """ Converts the desired pose to joint configuration using inverse kinematics """
    # Extract position and orientation from pose
    position, orientation = burg.util.position_and_quaternion_from_tf(desired_pose, convention='pybullet')
    # Calculate the joint configuration needed to set the end effector to the desired pose
    joint_positions = p.calculateInverseKinematics(robot, end_effector_link, position, orientation)
    # Use the joint positions for all controllable joints, not just the first 9
    joint_positions = list(joint_positions)[:7]
    return joint_positions

def move_robot(robot,start_conf, target_pose, collision_fn):
    """ Moves the robot from the start configuration to the target pose while checking for collisions """
    joint_positions = np.array(start_conf)
    # Convert target_pose to position and quaternion
    target_position, target_quaternion = burg.util.position_and_quaternion_from_tf(target_pose, convention='pybullet')
    # Calculate the joint positions needed to move end effector to the target position, keeping the same orientation
    target_joint_positions = p.calculateInverseKinematics(robot, 11, target_position, target_quaternion)
    target_joint_positions = np.array(target_joint_positions[:7])  # Ignore last 2 values for gripper

    # Generate waypoints from start_conf to end_conf
    num_waypoints = 20  # Define how many waypoints you want
    waypoints = np.linspace(joint_positions,target_joint_positions, num=num_waypoints)
    print("Calculated waypoints:", waypoints)

    for waypoint in waypoints:
        # Check for collision at the waypoint
        if collision_fn(waypoint):
            print("Collision detected! Stopping motion.")
            return False

        # Set joint positions
        for i, joint_position in enumerate(waypoint):
            print("Before resetting joint position:", p.getJointState(robot, i))
            p.resetJointState(robot, i, joint_position)
            print("After resetting joint position:", p.getJointState(robot, i))

        print("Moving to waypoint:", waypoint)
        # Wait for a while to let the robot move
        time.sleep(0.1)

    return True

def main():
    # Initialize Pybullet physics simulation
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path for PyBullet data
    p.setGravity(0, 0, -10)  # Set gravity in the simulation
    p.loadURDF("plane.urdf", [0, 0, 0])  # Load ground plane

    # Load Panda robot
    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    # Get all revolute joints
    revolute_joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
    print("Revolute joints:", revolute_joints)

    # Get the joint states and select only the revolute joints
    start_conf = p.getJointStates(robot, revolute_joints)
    start_conf = [conf[0] for conf in start_conf]
    print("Start configuration:", start_conf)

    # Create a collision checking function for the robot
    collision_fn = get_collision_fn(robot, revolute_joints, obstacles=[], self_collisions=True)

    # Define a goal pose for the robot
    goal_pose = np.array([
        [-1.0, 0.0, 0.0, 0.4],
        [0.0, 1.0, 0.0, 0.4],
        [0.0, 0.0, -1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Convert the goal pose to a joint configuration
    end_effector_link = 11
    end_conf = pose_to_joint_conf(robot, end_effector_link, goal_pose)
    print("Goal configuration:", end_conf)

    # Move the robot from the start configuration to the end configuration while checking for collisions
    successful = move_robot(robot,start_conf, goal_pose, collision_fn)
    if successful:
        print("Reached goal configuration without collisions.")
    else:
        print("Could not reach goal configuration due to collisions.")

    p.disconnect()  # Disconnect from the PyBullet simulator

if __name__ == "__main__":
    main()
