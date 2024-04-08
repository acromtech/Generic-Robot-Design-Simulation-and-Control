import numpy as np
import time
from Robot import Robot

PRISMATIC = 1
REVOLUTE = 0
X, Y, Z = 0, 1, 2

# ----------- USE A PYBULLET ROBOT (SIMULATION) ----------- 

# robot = Robot ()
# robot.launch_URDF_simulation("kuka_iiwa/model.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.view_mode()


# ----------- DESIGN YOUR OWN ROBOT THANKS TO DHM PARAMETERS (SIMULATION) ----------- 

# EXAMPLE : 4R ROBOT
# robot = Robot (joint_types  =[REVOLUTE, REVOLUTE, REVOLUTE, REVOLUTE],
#                 a           =[0,        2,        1.5,      1],
#                 alpha       =[np.pi/2,  0,        0,        0],
#                 d           =[0.4,      0,        0,        0],
#                 theta       =[np.pi/2,  0,        0,        0])
# robot.create_simplified_URDF_based_on_DHM_parameters()
# robot.launch_URDF_simulation("outfile.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.view_mode()


# ----------- VERIFY YOUR MODELISATION (SIMULATION) ----------- 

# robot = Robot()
# robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.set_endEffector_frame_offset(0.0,0.0,-0.111)
# robot.simulate.view_all_joint_frames()
# robot.simulate.view_all_link_frames()
# robot.simulate.view_endEffector_frame()
# robot.simulate.view_mode()


# ----------- POSITION JOINT CONTROL (SIMULATION) ----------- 

# robot = Robot(joint_distances=[0.2,0.15,0.127,0.111]) # [l1,l2,L1,L4]
# robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.move_joint(joint_index=1, displacement=0.2555,   max_speed=0.2, torque=100, acceptable_error=1E-5, wait_to_exit=True, plot_joint_positions=True)
# robot.simulate.move_joint(joint_index=1, displacement=0,        max_speed=0.2, torque=100, acceptable_error=1E-5, wait_to_exit=True, plot_joint_positions=True)
# time.sleep(1)


# ----------- POSITION ROBOT CONTROL (SIMULATION) USING YOUR MODELISATION ----------- 

robot = Robot(joint_distances=[0.2,0.15,0.127,0.111]) # [l1,l2,L1,L4]
robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
robot.move_robot_position(  X=0.2,    Y=0.2,  Z=0.2,  end_effector_orientation=1, velocity_percentage=1,  acceptable_error=1E-5,   wait_to_exit=True,   plot_joint_positions=True, plot_robot_traj=True)
robot.move_robot_position(  X=0.2,    Y=-0.1, Z=0.1,  end_effector_orientation=0, velocity_percentage=1,  acceptable_error=1E-5,   wait_to_exit=True,   plot_joint_positions=True, plot_robot_traj=True)
robot.move_robot_position(  X=-0.2,   Y=-0.2, Z=0.22, end_effector_orientation=0, velocity_percentage=1,  acceptable_error=1E-5,   wait_to_exit=True,   plot_joint_positions=True, plot_robot_traj=True)
robot.move_robot_position(  X=-0.1,   Y=0.3,  Z=0.18, end_effector_orientation=0, velocity_percentage=1,  acceptable_error=1E-5,   wait_to_exit=True,   plot_joint_positions=True, plot_robot_traj=True)
robot.move_robot_position(  X=0.1,    Y=0.25, Z=0.15, end_effector_orientation=0, velocity_percentage=1,  acceptable_error=1E-5,   wait_to_exit=True,   plot_joint_positions=True, plot_robot_traj=True)
time.sleep(1)


# ----------- POSITION ROBOT CONTROL (SIMULATION) AUTOMATIC MODELISATION ----------- 

# robot = Robot()
# robot.launch_URDF_simulation("./scara2/urdf/scara2.urdf", startPos=[0,0,0], fixedBase=True)
# robot.simulate.set_endEffector_frame_offset(0.0,0.0,-0.111)
# robot.simulate.view_endEffector_frame()
# robot.move_robot_position_auto(0.1,0.1,0.2,0,1)
# time.sleep(1)


# ----------- TRAJECTORY ROBOT CONTROL (SIMULATION) ----------- 

# ATTENTION CORRIGER LE FICHIER ! DEMANDER A AXEL DE REGENERER UNE TRAJ AVEC LE NOUVEAU MODELE
# robot = Robot()
# robot.launch_URDF_simulation("./scara/urdf/scara.urdf", startPos=[0,0,0], fixedBase=True)
# robot.move_robot_moveit_trajectory("./Tools/trajectory.txt")
# time.sleep(2)


# ----------- REAL ROBOT CONTROL ----------- 
# Use only if you have the USB-CAN Transceiver plugged
# CURRENTLY THIS PART ISN'T READY TO USE

# can_bus=CanBusGsUsb(0,1000000)
# can_bus.setup()

# j1 = MyActuatorRMD.L(1,1,can_bus)
# j2 = MyActuatorRMD.L(2,1,can_bus)
# j3 = MyActuatorRMD.L(3,1,can_bus)

# robot_3DoF = Robot (joints                  =[j1,j2,j3],
#                     q_config                =[0,0,0], 
#                     mechanical_features     =[10,20,15],  
#                     test_type               ="Real")