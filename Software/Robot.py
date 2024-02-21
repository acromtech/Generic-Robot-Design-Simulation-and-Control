# Install dependencies (You can comment this lines when libraries will installed)
import os
#os.system("pip3 install numpy")
#os.system("pip3 install matplotlib")

# For all
import string
import numpy as np
import matplotlib.pyplot as plt
# ... Add others library if necessary

# Robotic_Design test
from Tools.Tools import Tools
# ... Add others library if necessary

# Simulation test
from Simulation.Simulation import Simulation
# ... Add others library if necessary

# Real test
#from Real import MyActuatorRMD
#from Real import CanBusGsUsb
# ... Add others library if necessary

class Robot:
    
    def __init__(self, joints=[], joint_type=[], joint_distance=[], joint_orientation=[], q_config=[], URDF_file=None, test_type=""):
        """
        Parameters : 
            joints              :list[MyActuatorRMD]    # List of the robot joints - ex : [j1,j2,j3 ...]
            joint_type          :list[int]              # List of joint type (0=Prismatic, 1=Revolute) - ex : Robot RPPR -> [1,0,0,1]
            joint_distance      :list[float]            # List of contants - distance beetween each joints (in mm) - ex : [o0o1,o1o2,o2o3,...]
            joint_orientation   :list[int]              # List of contants - orientation of each joints (0=X, 1=Y, 2=Z) - ex : [0,0,1,...]
            q_config            :list[float]            # List of the q config of the joints - ex : [0,np.pi/2,0,...]
            URDF_file           :string                 # URDF file path - ex : "/path/to/the/URDF/file"
            test_type           :string                 # Test type - ("Robotic_Design", "Simulation" or "Real")
            
            os_master           :string                 # OS used - ("Linux" or "Windows")
            can_port            :int                    # Can port - ex : 0,1,2,3, ... (generaly 0 or 1)
            can_bitrate         :int                    # Can bitrate - ex : 500000 (500k), 1000000 (1M), ... (Myactuator joints setup on 1M)
        """
        # For a specific test ...
        self.test_type = test_type
        if (self.test_type=="Robotic_Design"):
            self.tools=Tools(joint_type,joint_distance,joint_orientation,q_config)
            # ... Run specfic method or initialise variable
            pass

        elif (self.test_type=="Simulation"):
            self.simulate=Simulation(URDF_file)
            # ... Run specfic method or initialise variable
            pass

        elif(self.test_type=="Real"):
            # self.can_bus = CanBusGsUsb(os_master,can_port,can_bitrate)
            # self.can_bus.setup()
            self.joints = joints
            print(self.joints)
            
        # ... Create others tests if you need it
        
        else:
            print("Error : Incorrect test_type, please choose 'Robotic_Design', 'Simulation' or 'Real'")

