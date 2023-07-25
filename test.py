import pybullet as p
import time
import math
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, 0])
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)

# Prepare a sequence of target positions
target_positions = [-0.4, 0.2, 0.2]

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

# PD gains
Kp = 1.5  # proportional gain
Kd = 0.3  # derivative gain

i = 0
hasPrevPose = 0
trailDuration = 15

while 1:
    target_pos = target_positions

    # end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])
    jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, target_pos, orn)

    for i in range(numJoints):
        joint_state = p.getJointState(kukaId, i)  # get the current state of the joint

        # compute the control input using the PD controller
        control_force = Kp * (jointPoses[i] - joint_state[0]) - Kd * joint_state[1]

        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.TORQUE_CONTROL,
                                force=control_force)

    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    if (hasPrevPose):
        p.addUserDebugLine(prevPose, target_pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = target_pos
    prevPose1 = ls[4]
    hasPrevPose = 1

    p.stepSimulation()
    time.sleep(0.05)

p.disconnect()
