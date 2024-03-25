# Code to create generic URDF from DH parameters
import os
import numpy as np
from numpy import sin, cos, pi

os.system('pip3 install scipy')
from scipy.spatial.transform import Rotation as R

import numpy as np

def generate_DH_parameters(num_joints, distances, orientations, joint_types):
    if len(distances) != num_joints - 1:
        print("Erreur : Le nombre de distances entre les joints doit être égal au nombre de joints moins un.")
        return None
    
    if len(orientations) != num_joints:
        print("Erreur : Le nombre d'orientations des joints doit être égal au nombre de joints.")
        return None
    
    if len(joint_types) != num_joints:
        print("Erreur : Le nombre de types de joints doit être égal au nombre de joints.")
        return None
    
    DH_params = []
    
    for i in range(num_joints):
        d = distances[i - 1] if i > 0 else 0
        a = distances[i] if i < num_joints - 1 else 0
        alpha = orientations[i]
        
        if joint_types[i] == 'revolute':
            DH_params.append(['r', d, a, alpha])
        elif joint_types[i] == 'prismatic':
            DH_params.append(['p', d, a, alpha])
        else:
            print("Erreur : Type de joint non reconnu pour le joint ", i)
            return None
    
    return DH_params

def DH_trans(DH, joint_val):
     d, theta, a, alpha = (0,0,0,0)
     if (DH[0] == 'r'): d, theta, a, alpha = (DH[1], joint_val, DH[2], DH[3])
     elif (DH[0] == 'p'): d, theta, a, alpha = (joint_val, DH[1], DH[2], DH[3])
     elif (DH[0] == 'f'): d, theta, a, alpha = (DH[1], DH[2], DH[3], DH[4])

     trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                           [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                           [0,          sin(alpha),               cos(alpha),               d           ],
                           [0,          0,                        0,                        1           ]])
     return trans_mat

def joint_transforms(DH_Params):
     transforms = []
     current_DOF = 0
     transforms.append(np.eye(4))
     for DH in DH_Params:
         if (DH[0] == 'r' or DH[0] == 'p'):
             transforms.append(DH_trans(DH, 0.0))
             current_DOF = current_DOF + 1
         else:
             transforms.append(DH_trans(DH, 0.0))
     return transforms

def joint_frames(transforms):
         joint_frames = [transforms[0]]
         for trans in transforms[1:]:
             joint_frames.append(joint_frames[-1] @ trans)
         return joint_frames

def xml_string(DH_Params, scale=1):
    outstring = ""
    transforms = joint_transforms(DH_Params)
    frames = joint_frames(transforms)
    outstring = outstring + "<robot name='robot'>\n"
    outstring = outstring + "\t<material name='blue'>\n\t\t<color rgba='0 0 0.8 1'/>\n\t</material>\n"
    outstring = outstring + "\t<material name='red'>\n\t\t<color rgba='0.8 0 0 1'/>\n\t</material>\n"

    for i in range(len(transforms) - 1):

        el = transforms[i]
        fr = frames[i]

        # We need to create a cylinder to represent the joint
        # If the index is not zero, connect it to the previous link
        # And a joint to connect it to the link
        # And a box to connect the joints

        rpy = R.from_matrix(fr[0:3,0:3]).as_euler('XYZ')
        
        outstring = outstring + "\t<link name='a{}'>\n".format(i)
        outstring = outstring + "\t\t<visual>\n"
        outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3])
        outstring = outstring + "\t\t\t<geometry>\n"
        outstring = outstring + "\t\t\t\t<cylinder length='1' radius='0.5'/>\n"
        outstring = outstring + "\t\t\t</geometry>\n"
        outstring = outstring + "\t\t\t<material name='blue'/>\n"
        outstring = outstring + "\t\t</visual>\n"
        
        # Adding inertial data
        # outstring += "\t\t<inertial>\n"
        # outstring += "\t\t\t<origin xyz='0 0 0'/>\n"
        # outstring += "\t\t\t<mass value='1'/>\n"
        # outstring += "\t\t\t<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/>\n"
        # outstring += "\t\t</inertial>\n"
        
        outstring = outstring + "\t</link>\n"

        # If not on the first transformation, fix the cylinder to the previous link
        if(i != 0):
            outstring = outstring + "\t<joint name='fix_a{}_to_l{}' type='fixed'>\n".format(i, i-1)
            outstring = outstring + "\t\t<parent link='l{}'/>\n".format(i-1)
            outstring = outstring + "\t\t<child link='a{}'/>\n".format(i)
            outstring = outstring + "\t\t<origin rpy='0 0 0' xyz='0 0 0'/>\n"
            outstring = outstring + "\t</joint>\n"

        # Add a cylinder that goes from the current origin to the next one
        origins_vector = transforms[i + 1][0:3,3]
        origins_vector_norm = np.linalg.norm(origins_vector)
        cylinder_origin = origins_vector/2
        rpy = [0, 0, 0]

        if (origins_vector_norm != 0.0):
            origins_vector_unit = origins_vector/origins_vector_norm
            axis = np.cross(origins_vector, np.array([0, 0, -1]))
            axis_norm = np.linalg.norm(axis)
            if (axis_norm != 0.0):
                axis = axis/np.linalg.norm(axis)
            angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))
            print("axis is {}".format(axis))
            print("angle is {}". format(angle))
            rpy = R.from_rotvec(angle * axis).as_euler('XYZ')

        outstring = outstring + "\t<link name='l{}'>\n".format(i)
        outstring = outstring + "\t\t<visual>\n"
        outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], cylinder_origin[0], cylinder_origin[1], cylinder_origin[2])
        outstring = outstring + "\t\t\t<geometry>\n"
        outstring = outstring + "\t\t\t\t<cylinder length='{}' radius='0.4'/>\n".format(origins_vector_norm) 
        outstring = outstring + "\t\t\t</geometry>\n"
        outstring = outstring + "\t\t\t<material name='red'/>\n"
        outstring = outstring + "\t\t</visual>\n"
        
        # Adding inertial data
        # outstring += "\t\t<inertial>\n"
        # outstring += "\t\t\t<origin xyz='0 0 0'/>\n"
        # outstring += "\t\t\t<mass value='1'/>\n"
        # outstring += "\t\t\t<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/>\n"
        # outstring += "\t\t</inertial>\n"
        
        outstring = outstring + "\t</link>\n"

        # Add the actual joint between the cylinder and link
        jointType = ""
        if (DH_Params[i][0] == 'r'): jointType = "continuous"
        elif (DH_Params[i][0] == 'p'): jointType = "prismatic"
        else: jointType = "fixed"

        outstring = outstring + "\t<joint name='move_l{}_from_a{}' type='{}'>\n".format(i, i, jointType)
        outstring = outstring + "\t\t<parent link='a{}'/>\n".format(i)
        outstring = outstring + "\t\t<child link='l{}'/>\n".format(i)
        outstring = outstring + "\t\t<axis xyz='{} {} {}'/>\n".format(fr[0,2], fr[1,2], fr[2,2])
        outstring = outstring + "\t\t<origin rpy= '0 0 0' xyz='{} {} {}'/>\n".format(el[0,3], el[1,3], el[2,3])   
        
        # Adding joint limits for prismatic joints
        if DH_Params[i][0] == 'p':
            outstring += "\t\t<limit effort='1000' velocity='0.5' lower='-3' upper='3' />\n"  # Adjust the limits as needed
        
        outstring = outstring + "\t</joint>\n" 

    outstring = outstring + "</robot>\n"
    return outstring

# DH Parameter Layout:
# ['r', d, a, alpha] for revolute joints
# ['p', theta, a, alpha] for prismatic joints
# ['f', d, theta, a, alpha] for fixed joints

# 5R ROBOT
DH_Params = []
DH_Params.append(['r', 0, 0, 0])
DH_Params.append(['r', 2, 0, np.pi/2])
DH_Params.append(['r', 0, 2, 0])
DH_Params.append(['r', 0, 2, 0])
DH_Params.append(['r', 0, 1, 0])

# num_joints = 5
# distances = [1, 2, 1, 3]  # Distances entre les joints
# orientations = [0, np.pi/2, 0, np.pi/2, 0]  # Orientations des joints (en radians)
# joint_types = ['revolute', 'prismatic', 'revolute', 'prismatic', 'revolute']  # Types de joints
# DH_Params = generate_DH_parameters(num_joints, distances, orientations, joint_types)
# if DH_Params is not None:
#     print("Paramètres DH générés avec succès :")
#     for i, param in enumerate(DH_Params):
#         print("Joint", i+1, ":", param)

f = open("/home/alexis/.local/lib/python3.10/site-packages/pybullet_data/outfile.urdf", "w")
f.write(xml_string(DH_Params))
f.close()

