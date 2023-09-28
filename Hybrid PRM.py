import pybullet as p
import time
import sys
import pybullet_data
import numpy as np
from scipy.stats import qmc
import burg_toolkit as burg
import pybullet_planning as pp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pybullet_planning.interfaces.robots.collision import get_collision_fn
from pybullet_planning.motion_planners import smooth_path
from pybullet_planning.motion_planners.utils import waypoints_from_path

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF("plane.urdf")
# Load the cupboard URDF
cupboard_path = "C:/Users/paul_/OneDrive - Aston University/Dessertation learning Process/cupboard.urdf"
position = [0.30, -0.3, 0]
orientation = p.getQuaternionFromEuler([3.14159 / 2, 0, 3.14159 / 2])  # 90 degrees pitch and -90 degrees yaw rotation
cupboard_id = p.loadURDF(cupboard_path, basePosition=position, baseOrientation=orientation, globalScaling=0.001,
                         useFixedBase=True)

robot = p.loadURDF("franka_panda/panda.urdf",[0,0,0],useFixedBase=True)
initial_joint_angles = [0.8214897102106269, -0.3627628274285913, 0.06249616451446385, -1.0671787864414042, 0.029315838964342797, 0.690071458587208, -1.4617677056424134]
for i, angle in enumerate(initial_joint_angles):
    p.resetJointState(bodyUniqueId=robot, jointIndex=i, targetValue=angle)

joint_indices = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]

obstacles = [cupboard_id]

def matrix_to_robot_config(robot, end_effector_link, matrix):
    # Convert 4x4 matrix to position and orientation using the burg toolkit
    position, orientation = burg.util.position_and_quaternion_from_tf(matrix, convention='pybullet')
    # Calculate the inverse kinematics to get joint configuration for the given pose
    joint_positions = p.calculateInverseKinematics(robot, end_effector_link, position, orientation)
    joint_positions = list(joint_positions)[:7]  # Assuming Franka Panda's 7 joint
    print(joint_positions)
    return joint_positions

goal_pose = np.array([
    [-1.0, 0.0, 0.0, 0.5],
    [0.0, 1.0, 0.0, -0.3],
    [0.0, 0.0, -1.0, 1.0],
    [0.0, 0.0, 0.0, 1.0]
])

cube_size = 0.2  # The length of a side of the cube
cube_center = np.array([0.6, 0, 0.6])  # Position of the center of the cube



half_size = cube_size * 2.5
corner_points = [
    cube_center + np.array([x, y, z])
    for x in [-half_size, half_size]
    for y in [-half_size, half_size]
    for z in [-half_size, half_size]
]
'''
# Cube visual and collision shape

cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size])
cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size], rgbaColor=[0, 1, 0, 0.5])  # Semi-transparent green cube
cube_body = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=cube_collision_shape,
                              baseVisualShapeIndex=cube_visual_shape,
                              basePosition=cube_center)
'''
def closest_point_in_segment(p, a, b):
    ab = b - a
    t = np.dot(p - a, ab) / np.dot(ab, ab)
    t = np.clip(t, 0, 1)
    closest_point = a + t * ab
    return closest_point

def segment_intersects_cube(a, b, cube_min, cube_max):
    for i in range(3):
        if (a[i] < cube_min[i] and b[i] < cube_min[i]) or (a[i] > cube_max[i] and b[i] > cube_max[i]):
            return False

    for i in range(3):
        if cube_min[i] <= a[i] <= cube_max[i] or cube_min[i] <= b[i] <= cube_max[i]:
            continue
        closest_on_segment = closest_point_in_segment(np.array([cube_min[i], 0, 0]), a, b)[i]
        if cube_min[i] <= closest_on_segment <= cube_max[i]:
            return True

    return False

def is_inside_cube(position):
    return all(cube_center[i] - half_size <= position[i] <= cube_center[i] + half_size for i in range(3))


GOAL_BIAS_PROBABILITY = 0.1  # 10% chance



#Intialize sobol sequence generator
sobol_seq = qmc.Sobol(len(joint_indices), scramble=True)
sobol_seq.random_state = np.random.RandomState(1)

def sample_fn():
    if np.random.uniform(0, 1) < GOAL_BIAS_PROBABILITY:
        return goal_joint_config

    #generate a single Sobol sequence sample
    while True:
        # Use Sobol sequence to generate a quasi-random sample
        sample = sobol_seq.random(n=1)[0]
        # Scale the sample to the joint ranges
        joint_ranges = [p.getJointInfo(robot, i)[8:10] for i in joint_indices]
        scaled_sample = [(high - low) * s + low for s, (low, high) in zip(sample, joint_ranges)]

        # Convert the joint sample to an end effector position
        end_effector_position, _ = p.getLinkState(robot, end_effector_link)[:2]

        if is_inside_cube(end_effector_position):
            return scaled_sample



def get_euclidean_distance_fn(weights):
    def distance_fn(q1, q2):
        diff = np.array(q2) - np.array(q1)
        return np.sqrt(np.dot(weights, diff * diff))
    return distance_fn

weights = [1, 1, 1, 1, 1, 1, 1]
distance_fn = get_euclidean_distance_fn(weights=weights)


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


collision_fn = get_collision_fn(robot, joint_indices, obstacles=obstacles,
                                self_collisions=True,
                                disabled_collisions=panda_self_collision_disabled_link_indices)



# Convert goal_pose into a joint configuration
end_effector_link = 11
goal_joint_config = matrix_to_robot_config(robot, end_effector_link, goal_pose)
# Check if the goal_joint_config is collision-free
if collision_fn(goal_joint_config):
    print("The goal configuration is in collision!")
    print("Please! give anothor location")
    sys.exit("Exiting due to collision in goal configuration.")
else:
    print("The goal configuration is collision-free!")


def is_inside_box(position, min_corner, max_corner):
    return all(min_corner[i] <= position[i] <= max_corner[i] for i in range(3))


def joint_to_cartesian(joint_angles, robot, end_effector_link):
    """
    Convert joint angles to the Cartesian position of the end effector.

    Parameters:
    - joint_angles: List of joint angles.
    - robot: The robot's body unique ID (from `p.loadURDF` or similar).
    - end_effector_link: The link index of the robot's end effector.

    Returns:
    - Cartesian position of the end effector as (x, y, z).
    """

    # Set the robot's joint angles
    for i, angle in enumerate(joint_angles):
        p.resetJointState(robot, i, angle)

    # Get the position of the end effector
    state = p.getLinkState(robot, end_effector_link)
    pos = state[0]  # The first item in the returned tuple is the Cartesian position

    return pos
def sample_line(segment, step_size=.02, min_corner=None, max_corner=None):
    (q1, q2) = segment

    # Convert q1 and q2 to Cartesian space
    p1 = joint_to_cartesian(q1,robot,end_effector_link)  # This function gives the end-effector position for joint config q1
    p2 = joint_to_cartesian(q2,robot,end_effector_link)

    if min_corner is not None and max_corner is not None:
        # Check the Cartesian segment against the cube
        if not segment_intersects_cube(np.array(p1), np.array(p2), np.array(min_corner), np.array(max_corner)):
            return

    diff = np.array(q2) - np.array(q1)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        q = tuple(np.array(q1) + l * diff / dist)
        end_effector_position, _ = p.getLinkState(robot, end_effector_link)[:2]

        # Check if the end effector position is inside the specified volume (box)
        if (min_corner is not None) and (max_corner is not None) and not is_inside_box(end_effector_position,
                                                                                       min_corner, max_corner):
            break
        yield q
    yield q2


def get_extend_fn(obstacles=[], min_corner=None, max_corner=None):
    collision_fn = get_collision_fn(robot, joint_indices, obstacles=obstacles,
                                    self_collisions=True,
                                    disabled_collisions=panda_self_collision_disabled_link_indices)

    roadmap = []

    def extend_fn(q1, q2):
        path = [q1]
        for q in sample_line(segment=(q1, q2), min_corner=min_corner, max_corner=max_corner):
            if collision_fn(q):
                return []
            path.append(q)
        return path

    return extend_fn, roadmap

min_corner = corner_points[0]
max_corner = corner_points[-1]


extend_fn, _ = get_extend_fn(obstacles, min_corner, max_corner)






path = pp.prm(start=initial_joint_angles, goal=goal_joint_config, distance_fn=distance_fn,sample_fn=sample_fn,
                             extend_fn=extend_fn, collision_fn=collision_fn,num_samples=300)


# Now compute the distance
distance = distance_fn(initial_joint_angles, goal_joint_config)
print("Distance between configurations:", distance)

smoothed_path = smooth_path(path, extend_fn, collision_fn, iterations=1)
print("Smoothed Path:", smoothed_path)


"""
    Extract waypoints from a path at a given resolution.

    Args:
    - path (list): A list of joint configurations forming the path.
    - resolution (int): Every 'resolution'-th configuration is taken as a waypoint.

    Returns:
    - list: Waypoints extracted from the path.
"""

waypoints = waypoints_from_path(smoothed_path, tolerance=0.001)


p.disconnect()

# Visualize the path
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.loadURDF("plane.urdf")
# Load the cupboard URDF
cupboard_path = "C:/Users/paul_/OneDrive - Aston University/Dessertation learning Process/cupboard.urdf"
position = [0.30, -0.3, 0]
orientation = p.getQuaternionFromEuler([3.14159 / 2, 0, 3.14159 / 2])  # 90 degrees pitch and -90 degrees yaw rotation
cupboard_id = p.loadURDF(cupboard_path, basePosition=position, baseOrientation=orientation, globalScaling=0.001,
                         useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf",[0,0,0],useFixedBase=True)
initial_joint_angles = [0.8214897102106269, -0.3627628274285913, 0.06249616451446385, -1.0671787864414042, 0.029315838964342797, 0.690071458587208, -1.4617677056424134]
for i, angle in enumerate(initial_joint_angles):
    p.resetJointState(bodyUniqueId=robot, jointIndex=i, targetValue=angle)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=2, cameraPitch=-40, cameraTargetPosition=[0, 0.3, 0.5])
time.sleep(2)

joint_indices = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]

obstacles = [cupboard_id]
'''
cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size])
cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_size, half_size, half_size], rgbaColor=[0, 1, 0, 0.5])  # Semi-transparent green cube
cube_body = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=cube_collision_shape,
                              baseVisualShapeIndex=cube_visual_shape,
                              basePosition=cube_center)
'''
time.sleep(0.05)
print(waypoints)
for conf in waypoints:
    for j, value in enumerate(conf):
        p.setJointMotorControl2(robot, joint_indices[j], p.POSITION_CONTROL, targetPosition=value)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)  # sleep to match the default pybullet timestep


# Assume samples is a list of your scaled_samples
samples = [sample_fn() for _ in range(300)]  # generating 1000 samples


# Extracting x, y, z coordinates from samples
x_coords = [sample[0] for sample in samples]
y_coords = [sample[1] for sample in samples]
z_coords = [sample[2] for sample in samples]

# Creating a new matplotlib figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plotting the points on the axes
ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')  # 'c' is the color and 'marker' is the shape of points

# Setting labels for the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Showing the plot
plt.show()

time.sleep(15)  # Pause to view the goal positionconda