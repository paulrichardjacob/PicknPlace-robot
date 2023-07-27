import pybullet as p
import pybullet_data
import pybullet_planning as pp
import numpy as np
import burg_toolkit as burg
import time
from pybullet_planning.interfaces.robots.collision import get_collision_fn

def pose_to_joint_conf(robot, end_effector_link, desired_pose):
    """ Converts the desired pose to joint configuration using inverse kinematics """
    position, orientation = burg.util.position_and_quaternion_from_tf(desired_pose, convention='pybullet')
    joint_positions = p.calculateInverseKinematics(robot, end_effector_link, position, orientation)
    joint_positions = list(joint_positions)[:7]
    return joint_positions

def move_robot(robot, start_conf, target_pose, collision_fn):
    joint_positions = np.array(start_conf)
    target_position, target_quaternion = burg.util.position_and_quaternion_from_tf(target_pose, convention='pybullet')
    target_joint_positions = p.calculateInverseKinematics(robot, 11, target_position, target_quaternion)
    target_joint_positions = np.array(target_joint_positions[:7])

    num_waypoints = 20
    waypoints = np.linspace(joint_positions,target_joint_positions, num=num_waypoints)
    print("Calculated waypoints:", waypoints)

    all_joint_positions = p.getJointStates(robot, [i for i in range(p.getNumJoints(robot))])
    all_joint_positions = [conf[0] for conf in all_joint_positions]

    for waypoint in waypoints:
        all_joint_positions[:7] = waypoint  # Update the positions of revolute joints
        if collision_fn(waypoint,diagnosis=True):  # Check collision for all joints
            print("Collision detected! Stopping motion.")
            return False

        for i, joint_position in enumerate(waypoint):
            print("Before resetting joint position:", p.getJointState(robot, i))
            p.resetJointState(robot, i, joint_position)
            print("After resetting joint position:", p.getJointState(robot, i))

        print("Moving to waypoint:", waypoint)
        time.sleep(0.1)

    return True


def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf", [0, 0, 0])

    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0.5], useFixedBase=True)

    all_joints = [i for i in range(p.getNumJoints(robot))]  # Get all joints
    print("All joints:", all_joints)

    revolute_joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
    print("Revolute joints:", revolute_joints)

    start_conf = p.getJointStates(robot, revolute_joints)
    start_conf = [conf[0] for conf in start_conf]
    print("Start configuration:", start_conf)

    collision_fn = get_collision_fn(robot, revolute_joints, obstacles=[], self_collisions=False)  # Check collision for all joints

    goal_pose = np.array([
        [-1.0, 0.0, 0.0, 0.3],
        [0.0, 1.0, 0.0, 0.4],
        [0.0, 0.0, -1.0, 0.8],
        [0.0, 0.0, 0.0, 1.0]
    ])

    end_effector_link = 11
    end_conf = pose_to_joint_conf(robot, end_effector_link, goal_pose)
    print("Goal configuration:", end_conf)

    successful = move_robot(robot, start_conf, goal_pose, collision_fn)
    if successful:
        print("Reached goal configuration without collisions.")
    else:
        print("Could not reach goal configuration due to collisions.")

    p.disconnect()

if __name__ == "__main__":
    main()
