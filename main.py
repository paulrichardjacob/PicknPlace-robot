import pybullet as p
import pybullet_data
import numpy as np
import burg_toolkit as burg
import pybullet_planning as pp
from pybullet_planning.interfaces.robots.collision import get_collision_fn


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf",[0,0,0],useFixedBase=True)

#setting the robot to intial position
desired_joint_angles = [0.5, 0.2, -0.3, -1.0, -0.5, 1.5, 0.5]
# Get the joint indices
joint_indices = range(len(desired_joint_angles))
# Set the robot to the desired configuration using setJointMotorControlArray
p.setJointMotorControlArray(bodyUniqueId=robot,jointIndices=joint_indices,controlMode=p.POSITION_CONTROL,targetPositions=desired_joint_angles)


#creating obstacle
obstacle = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05,0.05,0.05])
obstacle_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
obstacle_orientation = p.getQuaternionFromEuler([0, 0, 0]) # orientation
# Creating the first obstacle
obstacle_position1 = [0.5, -0.3, 0.5]
obstacle_body1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle, baseVisualShapeIndex=obstacle_visual, basePosition=obstacle_position1, baseOrientation=obstacle_orientation)

# Creating the second obstacle
obstacle_position2 = [0.5, 0.3, 0.5]
obstacle_body2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle, baseVisualShapeIndex=obstacle_visual, basePosition=obstacle_position2, baseOrientation=obstacle_orientation)

# Creating the third obstacle
obstacle_position3 = [0.5, 0, 0.3]
obstacle_body3 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle, baseVisualShapeIndex=obstacle_visual, basePosition=obstacle_position3, baseOrientation=obstacle_orientation)

obstacles = [obstacle_body1, obstacle_body2, obstacle_body3]

#define the revolute joints so that I can move that
revolute_joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
print("Revolute joints:", revolute_joints)

# Define the joint limits for Franka Panda
joint_limits = [p.getJointInfo(robot, joint)[8:10] for joint in revolute_joints]
def get_link_indices(robot, link_names):
    all_links = [p.getJointInfo(robot, i)[12].decode('UTF-8') for i in range(p.getNumJoints(robot))]
    return [all_links.index(name) for name in link_names]

panda_self_collision_disabled_link_names = [
        ('panda_link1', 'panda_link2'),
        ('panda_link2', 'panda_link3'),
        ('panda_link3', 'panda_link4'),
        ('panda_link4', 'panda_link5'),
        ('panda_link5', 'panda_link6'),
        ('panda_link6', 'panda_link7'),
        ('panda_link7', 'panda_link8')
    ]
panda_self_collision_disabled_link_indices = [get_link_indices(robot, pair)
                                                  for pair in panda_self_collision_disabled_link_names]

collision_fn = get_collision_fn(robot, revolute_joints, obstacles=obstacles,
                                    self_collisions=True,
                                    disabled_collisions=panda_self_collision_disabled_link_indices)
smooth = True
num_restarts = 0
max_time = 2


def matrix_to_robot_config(robot, end_effector_link, matrix):
    # Convert 4x4 matrix to position and orientation using the burg toolkit
    position, orientation = burg.util.position_and_quaternion_from_tf(matrix, convention='pybullet')

    # Calculate the inverse kinematics to get joint configuration for the given pose
    joint_positions = p.calculateInverseKinematics(robot, end_effector_link, position, orientation)
    joint_positions = list(joint_positions)[:7]  # Assuming Franka Panda's 7 joints

    # Print the configuration
    print("Joint Configuration:", joint_positions)

    return joint_positions

def get_delta(q1, q2):
    return np.array(q2) - np.array(q1)

def get_euclidean_distance_fn(weights):
    difference_fn = get_delta
    def distance_fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return distance_fn

weights = [1, 1, 1, 1, 1, 1, 1]
distance_fn = get_euclidean_distance_fn(weights=weights)


goal_pose = np.array([
    [-1.0, 0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0, 1.0],
    [0.0, 0.0, -1.0, 0.7],
    [0.0, 0.0, 0.0, 1.0]
])
end_effector_link=11
# Convert goal_pose into a joint configuration
goal_joint_config = matrix_to_robot_config(robot, end_effector_link, goal_pose)

# Now compute the distance
distance = distance_fn(desired_joint_angles, goal_joint_config)
print("Distance between configurations:", distance)



path = pp.birrt(start=desired_joint_angles, goal= goal_pose, distance_fn=distance_fn, sample_fn=sample_fn,
                             extend_fn=extend_fn, collision_fn=collision_fn,
                             max_time=max_time, smooth=100)


while True:
    p.stepSimulation()
