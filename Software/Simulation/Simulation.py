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

PRISMATIC = 1
REVOLUTE = 0
class Simulation:
    def __init__(self, urdf_file=None, startPos=[0,0,1], startOrientation=p.getQuaternionFromEuler([0,0,0]),fixedBase=False, viewMode=False):
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
        self.startOrientation=startOrientation
        self.fixedBase=fixedBase

        # Launch pybullet and import the simulation plane and the input urdf file
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=self.fixedBase)
        self.robot_id = p.loadURDF(f"{urdf_file}", self.startPos, self.startOrientation, useFixedBase=self.fixedBase)

        self.jointName = []
        self.jointType = []
        self.jointLowerLimit = []
        self.jointUpperLimit = []
        self.jointMaxForce = []
        self.jointMaxVelocity = []

        for i in range(p.getNumJoints(self.robot_id)):
            self.joint_info = p.getJointInfo(self.robot_id, i)
            self.jointName.append(self.joint_info[1].decode('utf-8'))
            self.jointType.append(self.joint_info[2])
            self.jointLowerLimit.append(self.joint_info[8])
            self.jointUpperLimit.append(self.joint_info[9])
            self.jointMaxForce.append(self.joint_info[10])
            self.jointMaxVelocity.append(self.joint_info[11])
            
        print("Names:       \t",self.jointName)
        print("Types:       \t",self.jointType)
        print("Lower Limits:\t",self.jointLowerLimit)
        print("Upper Limits:\t",self.jointUpperLimit)
        print("Max Force:   \t",self.jointLowerLimit)
        print("Max Velocity:\t",self.jointUpperLimit)
        
        if viewMode or urdf_file=="r2d2.urdf" : self.view_mode()
        elif (urdf_file=="kuka_iiwa/model.urdf"): self.kuka_simulation()
        elif (urdf_file=="outfile.urdf"): self.joint_test_dhm()
        elif (urdf_file=="./scara/urdf/scara.urdf"): self.joint_test()
        else: self.view_mode()

    def view_mode(self):
        while True:
            p.stepSimulation()
            time.sleep(1. / 240.)

    def kuka_simulation(self):
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        numJoints = p.getNumJoints(self.robot_id)
        if (numJoints != 7):
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

        for i in range(numJoints):
            p.resetJointState(self.robot_id, i, rp[i])
            self.move_joint(abs(j-5),i*0.1)

        for j in range (5):
            for i in range (100):
                p.stepSimulation()
                self.move_joint(abs(j-5),i*0.1)
                time.sleep(0.02)
        
        p.disconnect()

            # ... Add something here to move the joints of the kuka robot 
        
    def joint_test_dhm(self):
        move_joint_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name.startswith('move'):
                move_joint_indices.append(i)
        print(move_joint_indices)

        # Move all the joints
        for joint_index in move_joint_indices:
            print(f"test{joint_index}")
            for i in range(100):
                p.setJointMotorControl2(bodyUniqueId=self.robot_id,
                                        jointIndex=joint_index,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=i*0.01)
                p.stepSimulation()
                time.sleep(1./240)
            time.sleep(1)

        # Deconnexion
        p.disconnect()


    def joint_test(self):
        p.stepSimulation()

        # Move all the joints
        for joint_index in range(p.getNumJoints(self.robot_id)):
            print(p.getJointInfo(self.robot_id, joint_index))
            print(f"test{joint_index}")

            # Check if the joint has limits
            if self.jointLowerLimit[joint_index] == self.jointUpperLimit[joint_index]:
                self.move_joint(joint_index, 360)
                self.move_joint(joint_index, -360)
                self.move_joint(joint_index, 0)
            else:
                self.move_joint(joint_index, self.jointLowerLimit[joint_index] * 180/np.pi)
                self.move_joint(joint_index, self.jointUpperLimit[joint_index] * 180/np.pi)

        p.disconnect()


    def move_joint(self, joint_index, displacement, speed=1.0):
        """
        Move a joint of the robot to the target position.

        Parameters:xx
            joint_index (int): Index of the joint to move.
            displacement (float) :  distance in meter if it's a PRISMATIC joint 
                                    angle in degree if it's a REVOLUTE joint
            speed (float, optional): Speed at which to move the joint. Default is 1.0.
        """
        # Get current joint position
        current_position = p.getJointState(self.robot_id, joint_index)[0]

        if current_position != displacement:
            if self.jointType[joint_index] == REVOLUTE:
                angle_rad = displacement * np.pi / 180
                num_steps = 100
                for step in range(1, num_steps + 1):
                    # Interpolate between current position and target position
                    target_angle_rad = current_position + (angle_rad - current_position) * step / num_steps
                    p.setJointMotorControl2(bodyUniqueId= self.robot_id,
                                            jointIndex= joint_index,
                                            controlMode= p.POSITION_CONTROL,
                                            targetPosition= target_angle_rad,
                                            targetVelocity= speed)
                    p.stepSimulation()
                    time.sleep(1. / 240)
            elif self.jointType[joint_index] == PRISMATIC:
                num_steps = 100
                for step in range(1, num_steps + 1):
                    # Interpolate between current position and target position
                    target_position = current_position + (displacement - current_position) * step / num_steps
                    p.setJointMotorControl2(bodyUniqueId= self.robot_id,
                                            jointIndex= joint_index,
                                            controlMode= p.POSITION_CONTROL,
                                            targetPosition= target_position,
                                            targetVelocity= speed)
                    p.stepSimulation()
                    time.sleep(1. / 240)

    def move_joints(self, joint_indices, target_positions):
        """
        Move multiple joints of the robot to the target positions.

        Parameters:
            joint_indices (list[int]): Indices of the joints to move.
            target_positions (list[float]): Target positions for each joint.
        """
        control_modes = [p.POSITION_CONTROL] * len(joint_indices)
        forces = [500] * len(joint_indices)
        p.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                    jointIndices=joint_indices,
                                    controlMode=control_modes,
                                    targetPositions=target_positions,
                                    forces=forces)

        
    
    # ... Add methods if you need it
