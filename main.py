import pybullet as p
import pybullet_data
import numpy as np
import time

# Setup PyBullet
#def__init__(self):
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.loadURDF("plane.urdf", [0, 0, 0])

# Load Robot
Robot_arm = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
'''''
# Create target
box = [0.05, 0.05, 0.05]  # dimensions of box
object_pick = p.createCollisionShape(p.GEOM_BOX, halfExtents=box)
p.createMultiBody(baseMass=1, baseCollisionShapeIndex=object_pick, basePosition=[0.8, 0, 0])
'''
# Get target position and orientation
target_pos = np.array([0.8, 0.5, 0])  # Position of the box but a bit higher to avoid collision
target_orient = p.getQuaternionFromEuler([0, np.pi, 0])  # Rotate the gripper

# Simulation parameters
num_joints = 8
maxForce = 2000
maxVelocity = 1
kp = 2.0
kd = 0.5

while True:
    p.stepSimulation()

    target_joint_angles = p.calculateInverseKinematics(Robot_arm, num_joints, target_pos, target_orient)

    # Get joint states
    joint_states = p.getJointStates(Robot_arm, range(num_joints))
    current_joint_angles = [state[0] for state in joint_states]
    current_joint_velocities = [state[1] for state in joint_states]

    for i in range(num_joints):
        # Calculate the position and velocity errors
        position_error = target_joint_angles[i] - current_joint_angles[i]
        velocity_error = maxVelocity - current_joint_velocities[i]

        # Calculate control input (PD control)
        control_input = kp * position_error + kd * velocity_error

        p.setJointMotorControl2(bodyUniqueId=Robot_arm,
                                jointIndex=i,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=control_input,
                                force=maxForce)

    time.sleep(1. / 240.)
