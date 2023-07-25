import pybullet as p
import pybullet_data
import time
import math

# Starting the physics simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used to load plane.urdf

# Load the plane and the robot
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation)

# Create a box in the simulation
boxId = p.loadURDF("cube_small.urdf", [0.7,0,0.2])

# Enable the motors in the joints of the robot
for i in range(p.getNumJoints(robotId)):
    p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL,
                            targetPosition=0, targetVelocity=0,
                            force=500, positionGain=0.03, velocityGain=1)

# Set the desired end effector pose to grasp the box
# Note: These are dummy values. You should replace these with actual values
endEffectorPos = [0.7,0,0.3]
endEffectorOrn = p.getQuaternionFromEuler([0,math.pi,0])

# Loop for the control to reach the desired pose
while True:
    # Initialize some variables
    num_joints = p.getNumJoints(robotId)
    joint_states = p.getJointStates(robotId, range(num_joints))
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [0] * num_joints
    joint_accelerations = [0] * num_joints

    # This is the position on the robot where the Jacobian will be computed.
    # We compute it at the end effector, so we use the local position of the end effector.
    local_position = [0, 0, 0]

    # Calculate the Jacobian
    jac_t, jac_r = p.calculateJacobian(robotId,
                                       8,  # This is the link index for the end effector.
                                       local_position,
                                       joint_positions,
                                       joint_velocities,
                                       joint_accelerations)

    # Calculate the current pose of the end effector
    endEffectorState = p.getLinkState(robotId, 7)
    endEffectorPosCur = endEffectorState[0]
    endEffectorOrnCur = endEffectorState[1]

    # Calculate the control input
    posError = [a-b for a,b in zip(endEffectorPos, endEffectorPosCur)]
    ornError = [a-b for a,b in zip(endEffectorOrn, endEffectorOrnCur)]
    inputVec = posError + ornError
    controlInput = [sum(i) for i in zip(*[list(map(lambda x:x*jac_i, inputVec)) for jac_i in zip(*jac_t, *jac_r)])]

    # Apply control to the robot
    for i in range(p.getNumJoints(robotId)):
        p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL,
                                targetVelocity=controlInput[i], force=500)

    # Update the simulation
    p.stepSimulation()

    # Pause for a while
    time.sleep(0.01)

# Note: This example does not include grasp control.
# You need to add the code to control the gripper of the robot to grasp the box.
