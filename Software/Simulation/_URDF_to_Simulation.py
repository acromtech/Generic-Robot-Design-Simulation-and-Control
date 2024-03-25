import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print(pybullet_data.getDataPath())

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", useFixedBase=True)
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("outfile.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=True)

p.stepSimulation()

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)

move_joint_indices = []
for i in range(p.getNumJoints(boxId)):
    joint_info = p.getJointInfo(boxId, i)
    joint_name = joint_info[1].decode('utf-8')
    if joint_name.startswith('move'):
        move_joint_indices.append(i)
print(move_joint_indices)

for joint_index in move_joint_indices:
    print(f"test{joint_index}")
    for i in range(200):
        p.setJointMotorControl2(bodyUniqueId=boxId,
                                jointIndex=joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=i*0.01)
        p.stepSimulation()
        time.sleep(1./240)
    time.sleep(1)

p.disconnect()