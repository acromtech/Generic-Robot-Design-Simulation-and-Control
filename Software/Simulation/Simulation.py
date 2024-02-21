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
import pybullet as p
import pybullet_data
import math
import time
from datetime import datetime


class Simulation:
    def __init__(self, urdf_file=None, startPos=[0,0,1], startOrientation=p.getQuaternionFromEuler([0,0,0]),fixedBase=False):
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
        self.robot_id = p.loadURDF(f"{urdf_file}", self.startPos, self.startOrientation)

        # Run a specific simulation
        if (urdf_file=="r2d2.urdf"): self.r2d2_simulation()
        elif (urdf_file=="kuka_iiwa/model.urdf"): self.kuka_simulation()

    def r2d2_simulation(self):
        """
        Launch a test simulation.
        """
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1. / 240.)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robot_id)
        print(cubePos, cubeOrn)
        self.close()

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

        while True:
            p.stepSimulation()
            # ... Add something here to move the joints of the kuka robot

    def close(self):
        """
        Close the simulation.
        """
        p.disconnect()
        
    def reset_simulation(self):
        """
        Reset the simulation.
        """
        p.resetSimulation()


    # NEEDED METHODS
    def move_joint(self, joint_index, target_position):
        """
        Move a joint of the robot to the target position.

        Parameters:
            joint_index (int): Index of the joint to move.
            target_position (float): Target position to move the joint to.
        """
        
        
    def move_joints(self, joint_indices, target_positions):
        """
        Move multiple joints of the robot to the target positions.

        Parameters:
            joint_indices (list[int]): Indices of the joints to move.
            target_positions (list[float]): Target positions for each joint.
        """
        

    def get_joint_names(self):
        """
        Get the names of all joints in the URDF file.

        Returns:
            list[str]: List of joint names.
        """
        
    
    # ... Add methods if you need it
