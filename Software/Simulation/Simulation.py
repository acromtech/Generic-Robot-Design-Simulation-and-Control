"""
Simulation class for robotics simulation using PyBullet.

PyBullet is a fast and easy-to-use Python module for robotics simulation and machine learning.

Let’s see how to install pybullet on your Windows PC:

1. Verifying Python installation:
   - Open your command-line tool.
   - Run the command:
       py --version
   - If Python is installed, you should get output like "Python 3.x", where the number is your current Python version.
   - If you do not have Python, please install the latest 3.x version (2021) from python.org.

2. Making sure you can run pip commands:
   - Run the command:
       py -m pip --version
   - If it doesn’t return a version number of your installed pip, install it following any tutorial from the internet.

3. Installing Microsoft Visual Studio Build Tools:
   - Go to the download page for Microsoft Visual Studio Build Tools and download Build Tools for the last Visual Studio version 
        https://aka.ms/vs/17/release/vs_BuildTools.exe (click here to download the installer automaticly)
   - Open the installer and make sure "Desktop development with C++" is selected.
   - Click on the download/install/modify button.

If you have any problem check the following page : https://deepakjogi.medium.com/how-to-install-pybullet-physics-simulation-in-windows-e1f16baa26f6

4. When it's fully installed, uncomment the lines below to install pybullet:
import os
os.system("pip3 install --upgrade setuptools")
os.system("pip3 install pybullet")

When you successfully installed pybullet you can use the Simulation class as you want :
-> Firstly you can put "sim=Simulation()" in your main to launch a test code
You can check the pybullet's Github to learn more about the library : https://github.com/bulletphysics/bullet3?tab=readme-ov-file
"""
import os
import math
import time
#os.system('pip3 install pybullet')
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

PRISMATIC = 1
REVOLUTE = 0
class Simulation:
    def __init__(self, urdf_file=None, startPos=[0,0,1], startOrientation=[0,0,0],fixedBase=False, viewMode=False):
        """
        Initialize the simulation with a URDF file.

        Parameters:
            urdf_file (str): Path to the URDF file.
            startPos (list, optional): Starting position of the robot. Default is [0,0,1].
            startOrientation (list, optional): Starting orientation of the robot. Default is [0,0,0].
            fixedBase ...
        """
        # Initialize parameters
        self.urdf_file=urdf_file
        self.startPos=startPos
        self.startOrientation=p.getQuaternionFromEuler(startOrientation)
        self.fixedBase=fixedBase

        # Launch pybullet and import the simulation plane and the input urdf file
        self.physicsClient = p.connect(p.GUI)
        p.setRealTimeSimulation(1, self.physicsClient)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=self.fixedBase)
        self.robot_id = p.loadURDF(f"{urdf_file}", self.startPos, self.startOrientation, useFixedBase=self.fixedBase)

        self.numJoint = p.getNumJoints(self.robot_id)
        self.endEffectorIndex = self.numJoint-1
        self.jointName = []
        self.jointType = []
        self.jointLowerLimit = []
        self.jointUpperLimit = []
        self.jointMaxForce = []
        self.jointMaxVelocity = []
        self.jointFramePosition = []
        self.jointFrameOrientation = []

        self.endEffector_offset = [0.0, 0.0, 0.0]
        self.endEffector_frame = self.get_robot_current_position()

        for i in range(self.numJoint):
            self.joint_info = p.getJointInfo(self.robot_id, i)
            self.jointName.append(self.joint_info[1].decode('utf-8'))
            self.jointType.append(self.joint_info[2])
            self.jointLowerLimit.append(self.joint_info[8])
            self.jointUpperLimit.append(self.joint_info[9])
            self.jointMaxForce.append(self.joint_info[10])
            self.jointMaxVelocity.append(self.joint_info[11])
            self.jointFramePosition.append(self.joint_info[14])
            self.jointFrameOrientation.append(self.joint_info[15])

            print(f"Joint Frame{i} Position:    \t",self.jointFramePosition[i])
            print(f"Joint Frame{i} Orientation: \t",self.jointFrameOrientation[i])
        
        print("NumJoint:                \t",self.numJoint)
        print("EndEffectorIndex:        \t",self.endEffectorIndex)
        print("Names:                   \t",self.jointName)
        print("Types:                   \t",self.jointType)
        print("Lower Limits:            \t",self.jointLowerLimit)
        print("Upper Limits:            \t",self.jointUpperLimit)
        print("Max Force:               \t",self.jointLowerLimit)
        print("Max Velocity:            \t",self.jointUpperLimit)
        print("Start Position:          \t",self.startPos)
        print("Start Orientation:       \t",self.startOrientation)

        # self.fig, self.ax = plt.subplots(figsize=(10, 6))
        # self.joint_lines = {}  # Dictionnaire pour stocker les lignes des courbes de chaque joint
        # self.timestep = 0
        # self.fig.show()
        
        # if viewMode or urdf_file=="r2d2.urdf" : self.view_mode()
        # elif (urdf_file=="kuka_iiwa/model.urdf"): self.kuka_simulation()
        # elif (urdf_file=="outfile.urdf"): self.joint_test_dhm()
        # elif (urdf_file=="./scara/urdf/scara.urdf"): self.joint_test()
        # else: self.view_mode()

    def view_mode(self):
        print("------ Press ENTER to quit ------")
        if input(): pass

    def kuka_simulation(self):
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        if (self.numJoint != 7):
            exit()

        #lower limits for null space
        #ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
        #upper limits for null space
        #ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
        #joint ranges for null space
        #jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
        #restposes for null space
        rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
        #joint damping coefficents
        #jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        for i in range(self.numJoint):
            p.resetJointState(self.robot_id, i, rp[i])
            self.move_joint(abs(j-5),i*0.1)

        for j in range (5):
            for i in range (100):
                #p.stepSimulation()
                self.move_joint(abs(j-5),i*0.1)
                time.sleep(0.02)
        
        p.disconnect()

        # ... Add something here to move the joints of the kuka robot 
        
    def joint_test_dhm(self):
        move_joint_indices = []
        for i in range(self.numJoint):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name.startswith('move'):
                move_joint_indices.append(i)
        print(move_joint_indices)

        self.joint_test()
        self.view_mode()


    def joint_test(self):

        # Move all the joints
        for joint_index in range(self.numJoint):
            print(p.getJointInfo(self.robot_id, joint_index))
            print(f"test{joint_index}")

            # Check if the joint has limits
            if self.jointLowerLimit[joint_index] == self.jointUpperLimit[joint_index]:
                self.move_joint(joint_index     =joint_index, 
                                displacement    =360, 
                                max_speed       =1,
                                torque          =10,
                                wait_to_exit    =True)
                time.sleep(1)
                self.move_joint(joint_index     =joint_index, 
                                displacement    =-360, 
                                max_speed       =1,
                                torque          =10,
                                wait_to_exit    =True)
                time.sleep(1)
                self.move_joint(joint_index     =joint_index, 
                                displacement    =0, 
                                max_speed       =1,
                                torque          =10,
                                wait_to_exit    =True)
                time.sleep(1)
            else:
                self.move_joint(joint_index     =joint_index, 
                                displacement    =self.jointLowerLimit[joint_index] * 180/np.pi, 
                                max_speed       =10,
                                torque          =1000,
                                wait_to_exit    =True)
                time.sleep(1)
                self.move_joint(joint_index     =joint_index, 
                                displacement    =self.jointUpperLimit[joint_index] * 180/np.pi, 
                                max_speed       =10,
                                torque          =1000,
                                wait_to_exit    =True)
                time.sleep(1)

    def joints_test(self):
        self.move_joints(joint_indices      =[0,1,3],
                         displacements      =[90,0.10,360],
                         max_speeds         =[1,1,1],
                         torques            =[10,10,10],
                         positionGains      =[0.5,0.5,0.5],
                         velocityGains      =[0.255,0.255,0.255],
                         wait_to_exit       =False)
        time.sleep(10)
        
        self.move_joints(joint_indices      =[1,2],
                         displacements      =[0.20,360],
                         max_speeds         =[1,1],
                         torques            =[10,10],
                         positionGains      =[0.5,0.5],
                         velocityGains      =[0.255,0.255],
                         wait_to_exit       =False)
    
    # ------------ MOVE ROBOT JOINTS ------------

    def move_joint(self, joint_index, displacement, max_speed=1, torque=1000., error=1E-5, wait_to_exit=False):
        """
        Move a joint of the robot to the target position.

        Parameters:
            joint_index (int): Index of the joint to move.
            displacement        (float)     distance in meter if it's a PRISMATIC joint 
                                            angle in degree if it's a REVOLUTE joint
            max_speed           (float)     Speed at which to move the joint.       Default is 1.
            torque              (float)     Torque at which to move the joint.      Default is 10.
            positionGain        (float)     kp position gain of the joint           Default is 0.5
            velocityGain        (float)     kd velocity gain of the joint           Default is 0.225
        """
        if self.get_joint_angle_current_position(joint_index) != displacement:
            p.setJointMotorControl2(bodyIndex           =self.robot_id,
                                    jointIndex          =joint_index,
                                    controlMode         =p.POSITION_CONTROL,
                                    targetPosition      =displacement,
                                    maxVelocity         =max_speed,
                                    force               =torque)
            if wait_to_exit:
                while True :
                    if abs(self.get_joint_angle_current_position(joint_index) - displacement) < error: break


    def move_joints(self, joint_indices, displacements, max_speeds=None, torques=None, error=1E-5, wait_to_exit=False):
        """
        Move joints of the robot to the target positions.

        Parameters:
            joint_indices (list of int): Indices of the joints to move.
            displacements (list of float): Distances in meters if they are PRISMATIC joints, or angles in degrees if they are REVOLUTE joints.
            max_speeds (list of float): Speeds at which to move the joints. Default is [1.].
            torques (list of float): Torques at which to move the joints. Default is [10.].
            positionGains (list of float): kp position gains of the joints. Default is [0.5].
            velocityGains (list of float): kd velocity gains of the joints. Default is [0.225].
            wait_to_exit (bool): Whether to wait for the joints to reach their target positions before returning. Default is False.
        """
        if max_speeds==None: max_speeds=[1]*self.get_num_joint()
        if torques==None: torques=[100]*self.get_num_joint()

        p.setJointMotorControlArray(self.robot_id,joint_indices,p.POSITION_CONTROL,displacements,max_speeds,torques)

        if wait_to_exit:
            while True:
                positions = [self.get_joint_angle_current_position(joint_index) for joint_index in joint_indices]
                if all(abs(position - target) < error for position, target in zip(positions, displacements)):
                    break
    
    # ------------ MODEL CALCULATIONS ------------
    
    def calculate_inverse_dynamics(self, jointPositions=[], jointVelocities=[], jointAccelerations=[]):
        return p.calculateInverseDynamics(bodyUniqueId          =self.robot_id,
                                            objPositions        =jointPositions,
                                            objVelocities       =jointVelocities,
                                            objAccelerations    =jointAccelerations,
                                            physicsClientId     =self.physicsClient)
    
    def calculate_inverse_kinematics(self, targetPosition):
        return p.calculateInverseKinematics(bodyUniqueId           =self.robot_id,
                                            endEffectorLinkIndex   =self.endEffectorIndex,
                                            targetPosition         =targetPosition)
    
    # ------------ GET JOINTS INFO ------------

    def get_num_joint(self):
        return self.numJoint
    
    def get_type_joint(self, indice):
        return self.jointType[indice]
    
    def get_joint_angle_current_position(self,joint_indice):
        return p.getJointState(self.robot_id,joint_indice)[0]

    def get_joint_angle_current_positions(self):
        return [self.get_joint_angle_current_position(i) for i in range(self.get_num_joint())]
    
    def get_joint_frame_current_position(self, joint_index):
        return p.getJointInfo(self.robot_id, joint_index)[14]
    
    def get_joint_frame_current_positions(self):
        return [self.get_joint_frame_current_position(i) for i in range(self.get_num_joint())]
    
    # ------------ GET ROBOT INFO ------------
    
    def get_robot_current_position(self):
        data = p.getLinkState(bodyUniqueId=self.robot_id, linkIndex=self.endEffectorIndex, computeForwardKinematics=True)
        linkWorldPosition=data[0]
        linkWorldOrientation=data[1]
        worldLinkFramePosition=data[4]
        worldLinkFrameOrientation=data[5]
        return [linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation]
        #parent_frame_position = p.getJointInfo(self.robot_id, self.get_num_joint()-1)[14]
        #endEffector_position = [parent_frame_position[i] + self.endEffector_offset[i] for i in range(3)]
        #return endEffector_position
    
    def get_robot_current_position_all_link_frames(self):
        data = []
        linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation = [], [], [], []
        for i in range(self.endEffectorIndex):
            data = p.getLinkState(bodyUniqueId=self.robot_id, linkIndex=i, computeForwardKinematics=True)
            linkWorldPosition.append(data[0])
            linkWorldOrientation.append(data[1])
            worldLinkFramePosition.append(data[4])
            worldLinkFrameOrientation.append(data[5])
        return [linkWorldPosition, linkWorldOrientation, worldLinkFramePosition, worldLinkFrameOrientation]
    
    # ------------ OFFSET ------------

    def set_endEffector_frame_offset(self, X, Y, Z):
        self.endEffector_offset = [X, Y, Z]
        endEffector_data = self.get_robot_current_position()
        self.endEffector_frame = [endEffector_data[2][0] + X, endEffector_data[2][1] + Y, endEffector_data[2][2] + Z]
        # self.constraint_id = p.createConstraint(parentBodyUniqueId       =self.robot_id,
        #                                     parentLinkIndex         =self.get_num_joint()-1,
        #                                     childBodyUniqueId       =-1, # Aucun corps enfant (par rapport au monde)
        #                                     childLinkIndex          =-1, # Aucun lien enfant (par rapport au monde)
        #                                     jointType               =p.JOINT_FIXED, # Garder le joint fixe pour appliquer un décalage
        #                                     jointAxis               =[0, 0, 0], # Aucun axe de rotation
        #                                     parentFramePosition     =endEffector_data[2], # Avant d'appliquer l'offset
        #                                     childFramePosition      =self.endEffector_frame)  # Après application de l'offset
    
    def get_endEffector_frame(self):
        return self.endEffector_frame

    def get_endEffector_offset(self):
        return self.endEffector_offset

    # ------------ VIEW FRAMES ------------

    def update_frame_axes(self,frame_position, axis_length=0.5):
        p.addUserDebugLine(frame_position, [frame_position[0] + axis_length, frame_position[1], frame_position[2]], [1, 0, 0], 3, 0)
        p.addUserDebugLine(frame_position, [frame_position[0], frame_position[1] + axis_length, frame_position[2]], [0, 1, 0], 3, 0)
        p.addUserDebugLine(frame_position, [frame_position[0], frame_position[1], frame_position[2] + axis_length], [0, 0, 1], 3, 0)

    def delete_frames_axes(self):
        p.removeAllUserDebugItems()

    def view_link_frame(self, link_index, print_terminal=True):
        frame = self.get_robot_current_position_all_link_frames()[2][link_index]
        self.update_frame_axes(frame)
        if print_terminal: print(f"Link frame {link_index}:                   ",frame)

    def view_all_link_frames(self, print_terminal=True):
        for i in range(self.endEffectorIndex):
            self.view_link_frame(i, print_terminal)

    def view_joint_frame(self, joint_index, print_terminal=True):
        frame = self.get_joint_frame_current_position(joint_index)
        self.update_frame_axes(frame)
        if print_terminal: print(f"Joint frame {joint_index}:                  ",frame)
    
    def view_all_joint_frames(self, print_terminal=True):
        for i in range(self.get_num_joint()):
            self.view_joint_frame(i, print_terminal)

    def view_endEffector_frame(self, print_terminal=True):
        self.update_frame_axes(self.endEffector_frame)
        if print_terminal: print(f"End effector frame:                         ",self.endEffector_frame)

    # ------------ GENERATE GRAPHS ------------

    def update_plot_joint_positions(self):
        q = self.get_joint_angle_current_positions()
        self.timestep += 1
        for i, joint_position in enumerate(q):
            if i not in self.joint_lines:
                self.joint_lines[i], = self.ax.plot([], [], label=f"Joint {i}")
            self.joint_lines[i].set_data(range(self.timestep), joint_position)
        self.ax.set_xlabel('Temps')
        self.ax.set_ylabel('Position du joint')
        self.ax.set_title('Positions des joints au fil du temps')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        

    def update_plot_joint_speed(self):
        pass

    def update_plot_joint_torque(self):
        pass

    def plot_robot_position(self):
        pass
    
    def plot_robot_speed(self):
        pass

    def plot_robot_torque(self):
        pass
    
    