import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import os

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)


p.resetSimulation()


p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, 0.0])
urdf_path = os.path.join(os.getcwd(), "franka_description", "robots", "panda.urdf")
robotId = p.loadURDF(urdf_path, [0, 0, 0.5], useFixedBase = True)
#robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
#p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])
numJoints = p.getNumJoints(robotId)
robotEndEffectorIndex = numJoints-3  # numJoints-1

p.setGravity(0, 0, -9.81 )  #  -9.81 Earth gravity (m/s²)

urdfRoot = pybullet_data.getDataPath()
#tableId = p.loadURDF(os.path.join(urdfRoot,"table/table.urdf"), [-1,1,0.3],p.getQuaternionFromEuler([0,0,0]))
tableId = p.loadURDF("custom_table/table.urdf",[-0.9,0.6,0.0],p.getQuaternionFromEuler([0,0,0]))

p.changeDynamics(
        tableId,  # Body unique ID
        -1,  # The baseLink has an index of -1 in PyBullet.
        mass=10.0,  # New mass in kg
        localInertiaDiagonal=[1.0, 1.0, 1.0],  # Inertia matrix (diagonal elements)
    )

# Get the number of joints (links) in the model
numJointsTable = p.getNumJoints(tableId)


#print("Joint number:",numJoints )

## Robot Joint info
controllableJointIndices = []
# Print the names of all links
print(f"Robot has {numJoints } joints (links):")
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    linkName = jointInfo[12].decode('utf-8')  # Index 12 contains the joint/link name

    controllable = "FIXED"
    if jointInfo[2] != p.JOINT_FIXED:
      controllableJointIndices.append(i)      
      controllable = "Controllable"

    print(f"Link {i}: {linkName} ({controllable})")



shelfId = p.loadURDF("book_case/bookcase.urdf",[0.25,0.5,0.0],p.getQuaternionFromEuler([0,0,0]))

cube1_start_pos = [-0.35, 0.5, 0.7]  # Position (x, y, z)
cube_dim = [0.03, 0.03, 0.03]
cube1_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents = cube_dim )
cube1_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_dim , rgbaColor=[1, 0, 0, 1])  # Red color (RGBA)
mass= 2.0 #static box
cube1_id = p.createMultiBody(
    baseMass=mass,
    baseCollisionShapeIndex=cube1_shape,
    baseVisualShapeIndex=cube1_visual,
    basePosition=cube1_start_pos
)


cube2_start_pos = [0.65, 0.75, 1.6]  # Position (x, y, z)
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



#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
#rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0, 0, 0, 0, 0]
rp = [0.0]*numJoints 
#joint damping coefficents
jd = [0.1]*numJoints 

for i in range(numJoints):
  p.resetJointState(robotId, i, rp[i])


t = 0.0
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

t_4 = 150
rectangle_dist = 0.4
ref = [[-0.35, 0.1, 0.64 + rectangle_dist*i/t_4] for i in range(0, t_4)] + [[-0.35, 0.1 + rectangle_dist*i/t_4, 0.64 + rectangle_dist] for i in range(0, t_4)] + [[-0.35, 0.1 + rectangle_dist, 0.64 + rectangle_dist - rectangle_dist*i/(t_4)] for i in range(0, t_4)] + [[-0.35, 0.1 + rectangle_dist - rectangle_dist*i/t_4, 0.64] for i in range(0, t_4)]


j=0
while 1:
  j+=1
  #p.getCameraImage(320,
  #                 200,
  #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
  #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t += 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  for i in range(1):
    #pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]

    j_ref = j%(4*t_4) 
    pos = ref[j_ref]
    #end effector points down, not up (in case useOrientation==1)

    orn = p.getQuaternionFromEuler([0, math.pi, 0]) #-math.pi/2

    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(robotId,
                                                  robotEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(robotId,
                                                  robotEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
      else:
        jointPoses = p.calculateInverseKinematics(robotId,
                                                  robotEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)
    
    #print("jointPoses",len(jointPoses))
    # print("numJoints",numJoints)robotEndEffectorIndex
    #print(f"Joint Positions: {jointPoses}")
    if (useSimulation):
      
      for idx, i in enumerate(controllableJointIndices):
        p.setJointMotorControl2(bodyIndex=robotId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[idx],
                                targetVelocity=0,
                                force=1000,
                                positionGain=0.4,
                                velocityGain=1)
        
    else:
      #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
      for i in range(numJoints):
        for idx, i in enumerate(controllableJointIndices):
          p.resetJointState(robotId, i, jointPoses[idx])


    ## Control of the manipulator effector 

    # endEffector


  ls = p.getLinkState(robotId, robotEndEffectorIndex)
  if (hasPrevPose):
    p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
  prevPose = pos
  prevPose1 = ls[4]
  hasPrevPose = 1
p.disconnect()


# 