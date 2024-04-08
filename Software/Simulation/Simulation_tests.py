from Simulation import Simulation
from URDFGenerator import URDFGenerator
import pybullet_data
import numpy as np
# Check the pybullet_data directory
print(pybullet_data.getDataPath())

# First, read the text in the top of the Simulation.py file to install Pybullet on Windows

# After you install the pybullet lib, try this :
#sim=Simulation("r2d2.urdf")

# After, comment the code upward and uncomment the code below to run an example with the kuka arm
#sim=Simulation("kuka_iiwa/model.urdf", viewMode=True)  # You can modify the kuka_simulation method in the Simulation class

# After, try to simulate the Scara robot thanks to the URDF files and the code inside "kuka_simulation" 
# Interresting link : https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html
# Modify also the "move_joint", the "move_joints" and the "get_joint_names" methods, and add methods to symplify your code


# TRY TO CREATE A ROBOT WITH THE DHM TABLE
    # DH Parameter Layout:
    # ['r', d, a, alpha] for revolute joints
    # ['p', theta, a, alpha] for prismatic joints
    # ['f', d, theta, a, alpha] for fixed joints

# EXAMPLE : 5R ROBOT
# self.joint_distances[i]
# DH_Params.append(['r', 0, 0, 0])
# DH_Params.append(['r', 2, 0, np.pi/2])
# DH_Params.append(['r', 0, 2, 0])
# DH_Params.append(['r', 0, 2, 0])  
# DH_Params.append(['r', 0, 1, 0])

# u=URDFGenerator()
# u.create_URDF(DH_Params,"outfile.urdf")
# sim=Simulation(urdf_file="outfile.urdf", fixedBase=True, viewMode=False)

# NEXT YOU CAN MOVE THE REAL SCARA URDF FILE
sim=Simulation("./scara/urdf/scara.urdf", startPos=[0,0,0], fixedBase=True, viewMode=False)