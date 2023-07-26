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
    joint_limits = []
    num_joints = p.getNumJoints(robot)

    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        joint_limits.append((info[8], info[9]))  # lower and upper limit for each joint

    joint_limits = np.array(joint_limits)  # Convert to numpy array for convenience
    print(joint_limits)
    def sample_fn():
        q = np.zeros(num_joints)
        for i in range(num_joints):
            lower_limit, upper_limit = joint_limits[i]
            if np.isinf(lower_limit) or np.isinf(upper_limit):  # It's a continuous joint
                q[i] = np.random.uniform(-np.pi, np.pi)  # Sample from -pi to pi
            elif lower_limit == upper_limit:  # It's a fixed joint
                q[i] = lower_limit  # Or upper_limit, they're the same
            else:  # Regular joint
                q[i] = np.random.uniform(lower_limit, upper_limit)
        return q

    def extend_fn(start_conf, end_conf):
        num_steps = 100  # The number of intermediate steps between start_conf and end_conf
        for i in range(num_steps):
            t = float(i) / (num_steps - 1)  # The fraction of the way from start_conf to end_conf
            interpolated_conf = (1 - t) * np.array(start_conf) + t * np.array(end_conf)
            # Now you need to compute the IK solution to check if this configuration is possible
            joint_positions = p.calculateInverseKinematics(robot, 11, interpolated_conf[:3], interpolated_conf[3:])
            # Validate if this position is in collision
            if not is_in_collision(joint_positions):
                yield joint_positions

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
