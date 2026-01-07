import pybullet as p
import time
import pybullet_data
from math import *
import os


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version


#p.resetSimulation()

# Add the pybullet_data directory to the search path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#p.setGravity(0,0,-10)
p.setGravity(0, 0, -9.81)  # Earth gravity (m/s²)

planeId = p.loadURDF("plane.urdf")





# Get the absolute path to the URDF file
#urdf_path = os.path.join(os.getcwd(), "frankie.urdf")
urdf_path = os.path.join(os.getcwd(), "franka_description", "robots", "frankie.urdf")

print(urdf_path)
startPos = [0,0,0.04]
startOrientation = p.getQuaternionFromEuler([0,0,0])   # -3*pi/4
robotId = p.loadURDF(urdf_path,startPos, startOrientation)



tableId = p.loadURDF("custom_table/table.urdf",[-1,2,0.6],p.getQuaternionFromEuler([0,0,0]))


# Get the number of joints (links) in the model
numJoints = p.getNumJoints(tableId)

number_joints = p.getNumJoints(robotId)
#print("Joint number:",number_joints)

# Print the names of all links
print(f"Robot has {number_joints} joints (links):")
for i in range(number_joints):
    jointInfo = p.getJointInfo(robotId, i)
    linkName = jointInfo[12].decode('utf-8')  # Index 12 contains the joint/link name
    print(f"Link {i}: {linkName}")

p.changeDynamics(
        tableId,  # Body unique ID
        -1,  # The baseLink has an index of -1 in PyBullet.
        mass=8.0,  # New mass in kg
        localInertiaDiagonal=[1.0, 1.0, 1.0],  # Inertia matrix (diagonal elements)
    )


shelfId = p.loadURDF("book_case/bookcase.urdf",[1,2,0.5],p.getQuaternionFromEuler([0,0,0]))

cube1_start_pos = [1.5, -1, 0.5]  # Position (x, y, z)
cube_dim = [0.05, 0.05, 0.05]
cube1_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents = cube_dim )
cube1_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_dim , rgbaColor=[1, 0, 0, 1])  # Red color (RGBA)
mass= 1 #static box

#p.createMultiBody(mass,cube1_shape)
cube1_id = p.createMultiBody(
    baseMass=mass,
    baseCollisionShapeIndex=cube1_shape,
    baseVisualShapeIndex=cube1_visual,
    basePosition=cube1_start_pos
)


cube2_start_pos = [1, -1.5, 0.5]  # Position (x, y, z)
cube2_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation
cube2_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_dim)  # Half-extents for a 1m cube
cube2_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_dim, rgbaColor=[0, 1, 0, 1])  # Green color (RGBA)
cube2_mass = 1.0  # Mass in kg
cube2_id = p.createMultiBody(
    baseMass=cube2_mass,
    baseCollisionShapeIndex=cube2_shape,
    baseVisualShapeIndex=cube2_visual,
    basePosition=cube2_start_pos,
    baseOrientation=cube2_start_orientation,
)


#print(robotId)
#print(p.getJointInfo(robotId,0))
#print(p.getJointInfo(robotId,1))

speed = 10
amplitude = 0.8
jump_amp = 0.5
maxForce = 3.5
kneeFrictionForce = 0
kp = 1
kd = .5
maxKneeForce = 1000



mode = p.VELOCITY_CONTROL
# p.setJointMotorControl2(robotId, 0,  # number_joints-1,
#  	controlMode=mode, force=maxForce, targetVelocity = targetVel)

# p.setJointMotorControl2(robotId, 0, controlMode=p.POSITION_CONTROL,
#                             targetPosition=pi,
#                             positionGain=kp,
#                             velocityGain=kd,
#                             force=maxForce)


# p.setJointMotorControl2(robotId, 1, controlMode=p.POSITION_CONTROL,
#                             targetPosition=pi,
#                             positionGain=1,
#                             velocityGain=1,
#                             force=maxForce)


p.setJointMotorControlArray(robotId,[i for i in range(0,number_joints)], controlMode=p.POSITION_CONTROL,
                            targetPositions = [0 for i in range(0,number_joints)])
                              # force=maxForce

print(p.getLinkState(robotId,0))
print(p.getLinkState(robotId,number_joints-1))


print(p.getBaseVelocity(robotId,0))

#applyExternalForce/Torque


#
print(p.getNumBodies())
print(p.getBodyInfo(robotId))
#print(p.getBodyUniqueId)


t_f = 10000  #
for i in range (t_f):
    p.stepSimulation()

    if i> t_f//3 and i < t_f//3 + 2 :

        p.setJointMotorControlArray(robotId,[i for i in range(0,number_joints)], controlMode=p.POSITION_CONTROL,
                            targetPositions = [0.1 for i in range(0,number_joints)])


    time.sleep(1./240.)

time.sleep(1./10.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()


