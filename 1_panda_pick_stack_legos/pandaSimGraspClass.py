import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

detectLegoPosition = 0

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
#jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
jointPositions=[0.0, 0.0, 0.0, 0.0, 0.0, math.pi, 0.0, 0.0, 0.0]
rp = jointPositions




class PandaSim(object):
  def __init__(self, bullet_client, offset, orientation):
    self.bullet_client = bullet_client
    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
    self.offset = np.array(offset)
    self.orientation = orientation # = [-0.5, -0.5, -0.5, 0.5]

    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.legos=[]
    
    self.bullet_client.loadURDF("plane.urdf", [0, 0, 0])
    #self.bullet_client.loadURDF("custom_table/table.urdf",[-0.6,0.3,0.0],self.bullet_client.getQuaternionFromEuler([0,0,0]))
    self.bullet_client.loadURDF("tray/traybox.urdf",[-0.25,offset[1],0.0],self.bullet_client.getQuaternionFromEuler([0,0,0]), flags=flags)
    self.bullet_client.loadURDF("tray/traybox.urdf", [0+offset[0], 0+offset[1], -0.5+offset[2]], orientation , flags=flags)
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.1, 0.0, -0.4])+self.offset, flags=flags))
    self.bullet_client.changeVisualShape(self.legos[0],-1,rgbaColor=[1,0,0,1])
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.1, 0.0, -0.4])+self.offset, flags=flags))
    self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.0, 0.1, -0.4])+self.offset, flags=flags))
    self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, -0.1, -0.4])+self.offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [-0.1, 0.2, -0.4])+self.offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.1, 0.2, -0.4])+self.offset, flags=flags)
    orn = orientation #[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    eul = self.bullet_client.getEulerFromQuaternion(orientation)
    time.sleep(1./20.)
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([-0.1,-0.5,-0.5])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    self.state = 0
    self.control_dt = 1./240.
    self.finger_target = 0
    self.gripper_height = 0.2

    self.gripper_height_plus = 0.4  # 0.4
    self.gripper_height_minus = 0.00 #0.03 # 0.1
    self.legoId = 0

    #create a constraint to keep the fingers centered
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.

    self.prev_pos =  self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex-1, computeForwardKinematics=True)[4] 
    self.orn = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex-1, computeForwardKinematics=True)[5]
    # self.panda,
  
    self.addUserDebugFrame(self.prev_pos, 
        self.orn,
        size=0.2,  # Taille du repère
        rgbColor=[1, 0, 0],  # Couleur du repère (rouge dans cet exemple)
        lifeTime=0.05  # Durée de vie du repère en secondes (0 signifie qu'il reste indéfiniment)
        )

  def reset(self):
    pass

  def update_state(self):  #keyboardEvent
    keys = self.bullet_client.getKeyboardEvents()
    if len(keys)>0:
      for k,v in keys.items():
        if v&self.bullet_client.KEY_WAS_TRIGGERED:
          if (k==ord('1')):
            self.state = 1
          if (k==ord('2')):
            self.state = 2
          if (k==ord('3')):
            self.state = 3
          if (k==ord('4')):
            self.state = 4
          if (k==ord('5')):
                self.state = 5
          if (k==ord('6')):
                self.state = 6
        if v&self.bullet_client.KEY_WAS_RELEASED:
            self.state = 0

  def addUserDebugFrame(self,position, quaternion, size=0.2, rgbColor=[1, 0, 0], lifeTime=0):
    # Convertir le quaternion en matrice de rotation
    rotation_matrix = self.bullet_client.getMatrixFromQuaternion(quaternion)
    rotation_matrix = np.array(rotation_matrix).reshape(3, 3)

    # Définir les axes dans le repère local
    x_axis = rotation_matrix[:, 0]
    y_axis = rotation_matrix[:, 1]
    z_axis = rotation_matrix[:, 2]

    # Calculer les points finaux des axes
    x_end = position + x_axis * size
    y_end = position + y_axis * size
    z_end = position + z_axis * size

    # Dessiner les axes
    self.bullet_client.addUserDebugLine(position, x_end, [1, 0, 0], lineWidth=2, lifeTime=lifeTime)  # X-axis (rouge)
    self.bullet_client.addUserDebugLine(position, y_end, [0, 1, 0], lineWidth=2, lifeTime=lifeTime)  # Y-axis (vert)
    self.bullet_client.addUserDebugLine(position, z_end, [0, 0, 1], lineWidth=2, lifeTime=lifeTime)  # Z-axis (bleu)   

  def step_pile(self):

    pos_pile = np.array([-0.15,-0.05,self.gripper_height_plus])

    self.update_state()

    if self.state==6: # close finger
      self.finger_target = 0.001 # 0.01
    if self.state==5: # open finger
      self.finger_target = 0.03 
    self.bullet_client.submitProfileTiming("step")
 
    #print("self.state=",self.state)
    #print("self.finger_target=",self.finger_target)

    alpha = 0.96 #0.99  #smooth transition of the gripper heigh

    if self.state==1 or self.state==3 or self.state==4 or self.state==7 or self.state==8:
      
      if self.state==4:
      #gripper_height = 0.034
        self.gripper_height = alpha * self.gripper_height + (1.-alpha)*self.gripper_height_minus  #0.03

      if self.state==1 or self.state == 3:
        self.gripper_height = alpha * self.gripper_height + (1.-alpha)*self.gripper_height_plus
      
      if self.state == 7:
        self.gripper_height = alpha * self.gripper_height + (1.-alpha)*self.gripper_height_plus*0.5

      pos = [self.prev_pos[0], self.prev_pos[1] , self.gripper_height] # self.offset[2] +

      if self.state==8: # go to pile table
         pos = pos_pile

         #pos = pos.tolist()

      if self.state==1 or self.state == 3 or self.state== 4:

        if detectLegoPosition == 0:
          pos_lego, orn_lego = self.bullet_client.getBasePositionAndOrientation(self.legos[self.legoId]) # position of the lego 0
        #else: #visual servoing / estimation
        # to do

        if self.state == 3: #ascent 
          pos = [pos_lego[0], pos_lego[1] , 0 + self.gripper_height]
        else: # 4   #descent
          pos = [pos_lego[0], pos_lego[1] , pos_lego[2] + self.gripper_height]

      orn = self.bullet_client.getQuaternionFromEuler([0, math.pi, 0]) #math.pi/2.

      # smoothen trajectory
      pos = alpha*np.array(self.prev_pos) + (1-alpha)*np.array(pos)
      orn = 0.5*alpha*np.array(self.orn) + (1-0.5*alpha)*np.array(orn)

      self.prev_pos = pos
      self.orn = orn
      self.bullet_client.submitProfileTiming("IK")
 
      jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
        jr, rp, maxNumIterations=20)
      self.bullet_client.submitProfileTiming()

      #target for panda dof / joints
      for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force= 100) # 5*240  

        
    #target for fingers
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target,force= 5) #10

    self.bullet_client.submitProfileTiming()

    self.addUserDebugFrame(self.prev_pos, 
        self.orn,
        size=0.2,  # Taille du repère
        rgbColor=[1, 0, 0],  # Couleur du repère (rouge dans cet exemple)
        lifeTime=0.05  # Durée de vie du repère en secondes (0 signifie qu'il reste indéfiniment)
        )

class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, offset, orientation):
    PandaSim.__init__(self, bullet_client, offset, orientation)
    self.state_t = 0
    self.cur_state = 0
    self.states = [0,3,5,4,6,3,8,7,5]
    #self.state_durations = [1,1,1,2,1,1, 10]  # 10
    #self.state_durations = [3,3,1,1,1,2,2,2,2] 
    self.state_durations = [0.5,0.5,0.5,0.8,0.5,0.5,0.5,0.5,0.5] 
  
  def update_state(self):
    # automatically handle the robot state shift with the time durations
    #according to 
    #self.states : the sequence of state which the manipulator must follow
    #self.state_durations : the corresponding duration for each state in the sequence

    self.state_t += self.control_dt # time duration on a state
    time.sleep(1./500.) #to slow the sim
    if self.state_t > self.state_durations[self.cur_state]:
      self.cur_state += 1
      if self.cur_state >= len(self.states):
        self.cur_state = 0
        self.legoId += 1

        if self.legoId >= len(self.legos):
          print("Stack assembly task completed !")
          time.sleep(1./100.)
          self.bullet_client.disconnect()

      self.state_t = 0
      self.state=self.states[self.cur_state]
      print("self.state=",self.state)

      

