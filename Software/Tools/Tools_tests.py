import numpy as np
from Tools import Tools

P,R=0,1         # P=0, R=1          WARNING : NO MODIFY THESE VALUES. These values are sets to better understanding the code below
X,Y,Z=0,1,2     # X=0, Y=1, Z=2     WARNING : NO MODIFY THESE VALUES. These values are sets to better understanding the code below

o0o1,o1o2,o2o3,o3o4=10,10,10,10     # You can modify these values as you want

# SCARA FOR CLASSIC USE-CASE
robotRRRP_1 = Tools(joint_distances     =[o0o1,o1o2,o2o3,o3o4],
                    q_config            =[0,np.pi/2,0,0])

print("\n\nDHM PARAMETERS ====================================================================================")
robotRRRP_1.set_DHM_parameters(a        =[0,1,3,5],     # Set list of a_{i-1} DHM parameters
                               alpha    =[5,61,9,8],    # Set list of alpha_{i-1} DHM parameters
                               r        =[10,56,7,2])   # Set list of r_i DHM parameters

print("\nGET DHM PARAMETERS")
print(robotRRRP_1.get_DHM_parameters('a'))  # Get and display 'a' DHM parameter
print(robotRRRP_1.get_DHM_parameters('r'))  # Get and display 'r' DHM parameter
print(robotRRRP_1.get_DHM_parameters())     # Get and display all DHM parameters

print("\n\nJOINTS PARAMETERS ====================================================================================")
print("\nSET JOINTS PARAMETERS")
robotRRRP_1.set_joint_parameters(joint_distances=[None,None,12,None])       # Set one or more parameters as you want
robotRRRP_1.set_joint_parameters(q_config=[None,0,None,None])

print("\nGET JOINTS PARAMETERS")
print(robotRRRP_1.get_joint_parameters('joint_distances'))                  # Get and display a single parameter (here 'joint_distances')
print(robotRRRP_1.get_joint_parameters(['joint_distances','q_config']))     # Get and display 2 or more parameters (here 'joint_distances' and 'q_config')
print(robotRRRP_1.get_joint_parameters())                                   # Get and display all joint parameters

print("\n\nTRAJECTOTY ====================================================================================")
print("\nSET TRAJECTORY")
robotRRRP_1.set_trajectory_parameters(A=[10,50,20],V1=10.)  # Set one or more parameters as you want
robotRRRP_1.set_trajectory_parameters(B=[40,2,56])

print("\nGET TRAJECTORY")
print(robotRRRP_1.get_trajectory_parameters('A'))           # Get and display a single parameter (here 'A')
print(robotRRRP_1.get_trajectory_parameters(['V1','V2']))   # Get and display 2 or more parameters (here 'V1' and 'V2')
print(robotRRRP_1.get_trajectory_parameters())              # Get and display all trajectory parameters

print("\nTRAJECTORY CALCULATIONS")
robotRRRP_1.trajectory_calculations()                       # Compute the trajectory with the set parameters (Each parameters are optional)
robotRRRP_1.trajectory_calculations(V1=4)                   # Compute the trajectory with the set parameters and V1=4 (Each parameters are optional)

# ...

# SCARA FOR AUTOMATIC DHM PARAMETERS CALCULATIONS
robotRRRP_2 = Tools(joint_types         =[R,R,R,P], 
                    joint_distances     =[o0o1,o1o2,o2o3,o3o4], 
                    joint_orientations  =[X,X,Z,Y], 
                    q_config            =[0,np.pi/2,0,0])