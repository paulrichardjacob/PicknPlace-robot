import pybullet as p
import pybullet_data
import numpy as np
import time

# Setup PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.loadURDF("plane.urdf", [0, 0, 0])

# Load Robot
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Create target object
object_place = np.array([0.5,-0.5,0])
target_position = object_place + np.array([0, 0, 0.22])  # slightly above the ground to grasp it better
# Target Orientation - perpendicular to the box
target_ori = [0,0,0,1]  # 90 degrees rotation around x-axis
target_object = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
target_object = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=target_object, basePosition=object_place.tolist())

num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    print(joint_info)

# Control Parameters
maxForce = 2000
Kp = 2.0  # proportional gain
Kd = 0.2  # derivative gain
prev_errors = [0] * 7  # errors from previous step
gripper_closed = False  # to keep track of whether gripper is closed
gripper_open = 0.04  # position for opened gripper
gripper_close = 0  # position for closed gripper

# Simulation parameters
simulation_step = 1./240.

# Open the gripper
p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=gripper_open, force=maxForce)
p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=gripper_open, force=maxForce)

check_links = list([1,2,3,4,5,6,11])
print(check_links)

# Control loop
while True:
    # Get end effector position and orientation
    end_effector_state = p.getLinkState(robot, 11)
    end_effector_pos = np.array(end_effector_state[0])
    end_effector_orient = np.array(end_effector_state[1])

    # Use target_ori in your IK calculation
    target_joint_angles = p.calculateInverseKinematics(robot, 11, target_position, target_ori, maxNumIterations=100,
                                                       lowerLimits=[-3]*7, upperLimits=[3]*7, jointRanges=[6]*7)  # consider only first 7 joints
    target_joint_angles = list(target_joint_angles)[:7]  # consider only first 7 joint angles

    # Get the joint state
    joint_states = p.getJointStates(robot, check_links)

    # Iterate through each joint and set the control
    for i in range(7):
        joint_pos = joint_states[i][0]
        joint_vel = joint_states[i][1]
        joint_target = target_joint_angles[i]

        # Compute the error
        error = joint_target - joint_pos

        # Compute the derivative of the error
        error_derivative = (error - prev_errors[i]) / simulation_step

        # Compute the PD control
        control_force = Kp * error + Kd * error_derivative
        control_force = np.clip(control_force, -maxForce, maxForce)

        # Set the joint motor
        p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, targetVelocity=control_force, force=maxForce)

        # Store the current error for the next loop
        prev_errors[i] = error

    # Step the simulation
    p.stepSimulation()
    time.sleep(simulation_step)
