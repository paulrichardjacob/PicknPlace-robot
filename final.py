import pybullet as p
import pybullet_data
import pybullet_planning as pp
import numpy as np
import burg_toolkit as burg
import time

def main():
    # Initialize Pybullet physics simulation
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path for PyBullet data
    p.setGravity(0, 0, -10)  # Set gravity in the simulation
    p.loadURDF("plane.urdf", [0, 0, 0])  # Load ground plane

    # Load Panda robot
    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    def set_initial_pose(robot, desired_pose):
        # Convert the pose to a position and a quaternion
        initial_position, initial_quaternion = burg.util.position_and_quaternion_from_tf(desired_pose,
                                                                                         convention='pybullet')

        # Calculate the joint positions needed to set the end effector to the initial pose
        initial_joint_positions = p.calculateInverseKinematics(robot, 11, initial_position, initial_quaternion)
        initial_joint_positions = np.array(initial_joint_positions[:9])  # Ignore last 2 values for gripper

        # Set the joint positions
        for i in range(9):  # There are 9 joints for the robot arm and gripper
            p.resetJointState(robot, i, initial_joint_positions[i])

        p.stepSimulation()  # Update the simulation
        time.sleep(5)  # Sleep to make sure simulation updates

    # Set an initial pose for the robot
    desired_pose = np.array([
        [-1.0, 0.0, 0.0, 0.2],
        [0.0, 1.0, 0.0, 0.3],
        [0.0, 0.0, -1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0]
    ])
    set_initial_pose(robot, desired_pose)

    p.disconnect()  # disconnect from the PyBullet simulator


if __name__ == "__main__":
    main()
