import burg_toolkit as burg
import numpy as np

world_coordinate_system = np.eye(4)

my_coordinate_system = np.asarray([
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 0.5],
    [0.0, 0.0, 0.0, 1.0]
])

# you can convert them into the corresponding position and quaternions
# quaternion has four elements, pybullet uses the order x y z w, some other packages use w x y z (convention)
pos, quat = burg.util.position_and_quaternion_from_tf(world_coordinate_system, convention='pybullet')
print('world frame position and quaternion:', pos, quat)

pos, quat = burg.util.position_and_quaternion_from_tf(my_coordinate_system, convention='pybullet')
print('my frame position and quaternion:', pos, quat)

# let's visualise the frames so you can see what's happening
world_frame = burg.visualization.create_frame(size=0.1, pose=world_coordinate_system)
my_frame = burg.visualization.create_frame(size=0.1, pose=my_coordinate_system)
ground_plane = burg.visualization.create_plane()

burg.visualization.show_geometries([ground_plane, world_frame, my_frame])
