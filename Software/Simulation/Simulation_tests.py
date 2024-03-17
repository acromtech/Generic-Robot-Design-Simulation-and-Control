from Simulation import Simulation

# First, read the text in the top of the Simulation.py file to install Pybullet on Windows

# After you install the pybullet lib, try this :
sim=Simulation("r2d2.urdf")

# After, comment the code upward and uncomment the code below to run an example with the kuka arm
#sim=Simulation("kuka_iiwa/model.urdf")  # You can modify the kuka_simulation method in the Simulation class

# After, try to simulate the Scara robot thanks to the URDF files and the code inside "kuka_simulation" 
# Interresting link : https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html
# Modify also the "move_joint", the "move_joints" and the "get_joint_names" methods, and add methods to symplify your code
