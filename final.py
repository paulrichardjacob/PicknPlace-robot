import pybullet as p
import pybullet_data
import pybullet_planning as pp
import numpy as np

def main():
    # Initialize Pybullet physics simulation
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add search path for PyBullet data
    p.setGravity(0, 0, -10)  # Set gravity in the simulation
    p.loadURDF("plane.urdf", [0, 0, 0])  # Load ground plane

    # Load Panda robot
    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    joints = [j for j in pp.get_joints(robot) if pp.has_joint_limits(j)] # get the robot joints
    end_effector_link = pp.get_link_info(robot, 7)

    # Set the start conf to be the current configuration
    start_conf = pp.get_joint_positions(robot, joints)

    # Set the end conf to be the desired target position
    # NOTE: In a real scenario, you would calculate the joint angles that would result in the desired end effector pose using inverse kinematics.
    end_conf = start_conf.copy()
    end_conf[2] += np.pi / 2  # Change one of the joint positions as an example

    obstacles = [] # this can be populated with obstacle geometries if needed

    # Define the relevant functions for the motion planner
    sample_fn = pp.get_sample_fn(robot, joints)
    extend_fn = pp.get_extend_fn(robot, joints)
    distance_fn = pp.get_distance_fn(robot, joints)
    collision_fn = pp.get_collision_fn(robot, joints, obstacles=obstacles, attachments=[])

    # Call the RRT planner
    path = pp.rrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn)

    # Visualize the path if found and 
    if path is not None:
        print("Path found!")
        for conf in path:
            pp.set_joint_positions(robot, joints, conf)
            p.stepSimulation()
    else:
        print("No path found.")

    p.disconnect()  # disconnect from the PyBullet simulator

if __name__ == "__main__":
    main()
