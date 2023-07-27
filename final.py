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
    robot_joints = [i for i in range(p.getNumJoints(robot))]
    joint_limits = []
    num_joints = p.getNumJoints(robot)

    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        joint_limits.append((info[8], info[9]))  # lower and upper limit for each joint

    joint_limits = np.array(joint_limits)  # Convert to numpy array for convenience
    print(joint_limits)

    # Define the euclidean distance function
    def get_delta(q1, q2):
        return np.array(q2) - np.array(q1)

    def get_euclidean_distance_fn(weights):
        difference_fn = get_delta

        def distance_fn(q1, q2):
            diff = np.array(difference_fn(q2, q1))
            return np.sqrt(np.dot(weights, diff * diff))

        return distance_fn

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

    # Getting collision function
    collision_fn = pp.get_collision_fn(robot, robot_joints, obstacles=[],
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    def extend_fn(start_conf, end_conf):
        num_steps = 100  # The number of intermediate steps between start_conf and end_conf
        for i in range(num_steps):
            t = float(i) / (num_steps - 1)  # The fraction of the way from start_conf to end_conf
            interpolated_conf = (1 - t) * np.array(start_conf) + t * np.array(end_conf)
            # Now you need to compute the IK solution to check if this configuration is possible
            joint_positions = p.calculateInverseKinematics(robot, end_effector_link, interpolated_conf[:3], interpolated_conf[3:])
            # Validate if this position is in collision
            if not collision_fn(joint_positions):
                yield joint_positions

    distance_fn = get_euclidean_distance_fn(weights=np.ones(num_joints))

    def pose_to_joint_conf(robot, end_effector_link, desired_pose):
        # Extract position and orientation from pose
        position, orientation = burg.util.position_and_quaternion_from_tf(desired_pose, convention='pybullet')

        # Calculate the joint configuration needed to set the end effector to the desired pose
        joint_positions = p.calculateInverseKinematics(robot, end_effector_link, position, orientation)
        joint_positions = list(joint_positions)
        return joint_positions

    def set_initial_pose(robot, desired_pose):
        # Convert the pose to a position and a quaternion
        initial_position, initial_quaternion = burg.util.position_and_quaternion_from_tf(desired_pose,
                                                                                         convention='pybullet')

        #Calculate the joint positions needed to set the end effector to the initial pose
        initial_joint_positions = p.calculateInverseKinematics(robot, end_effector_link, initial_position, initial_quaternion)
        initial_joint_positions = np.array(initial_joint_positions)
         #Set the joint positions
        for i in range(end_effector_link):  # There are 9 joints for the robot arm and gripper
            p.resetJointState(robot, i, initial_joint_positions[i])

        p.stepSimulation()  # Update the simulation
        time.sleep(5)  # Sleep to make sure simulation updates

    # Set an initial pose for the robot
    desired_pose = np.array([
        [-1.0, 0.0, 0.0, -0.6],
        [0.0, 1.0, 0.0, 0.2],
        [0.0, 0.0, -1.0, 0.5],
        [0.0, 0.0, 0.0, 1.0]
    ])
    #set_initial_pose(robot, desired_pose)

    # Convert the desired pose to a joint configuration
    end_effector_link = 12
    start_conf = pose_to_joint_conf(robot, end_effector_link, desired_pose)
    print("Desired configuration:", start_conf)

    # Define a goal pose for the robot
    goal_pose = np.array([
        [-1.0, 0.0, 0.0, 0.5],
        [0.0, 1.0, 0.0, 0.4],
        [0.0, 0.0, -1.0, 0.6],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Convert the goal pose to a joint configuration
    end_conf = pose_to_joint_conf(robot, end_effector_link, goal_pose)
    print("Goal configuration:", end_conf)
    max_time = 20
    path = pp.birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn,
                  max_time=max_time)
    print(path)
    print("Is initial configuration in collision? ", collision_fn(start_conf))
    print("Is goal configuration in collision? ", collision_fn(end_conf))

    p.disconnect()  # disconnect from the PyBullet simulator


if __name__ == "__main__":
    main()