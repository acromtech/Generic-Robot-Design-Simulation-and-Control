import time
import numpy as np

from Tools.Tools import Tools
from Simulation.Simulation import Simulation
from Simulation.URDFGenerator import URDFGenerator
from Real.MyActuatorRMD import MyActuatorRMD
from Real.CanBusGsUsb import CanBusGsUsb

PRISMATIC = 1
REVOLUTE = 0

class Robot:
    
    def __init__(self, joint_types=[], joint_distances=[], joint_orientations=[], d=[], alpha=[], a=[], theta=[]):
        """
        Parameters : 

            IF YOU CHOOSE Robotic_Design (without dynamics)
            joint_types         :list[int]              # List of joint type (0=Prismatic, 1=Revolute) - ex : Robot RPPR -> [1,0,0,1]
            joint_distances     :list[float]            # List of contants - distance beetween each joints (in mm) - ex : [o0o1,o1o2,o2o3,...]
            joint_orientations  :list[int]              # List of contants - orientation of each joints (0=X, 1=Y, 2=Z) - ex : [0,0,1,...]
            
            IF YOU CHOOSE Simulation (with dynamics)
            urdf_file           :string                 # URDF file path - ex : "/path/to/the/URDF/file"
            startPos            :list[float]            # Starting position of the robot. Default is [0,0,1].
            startOrientation    :list[float]            # Starting orientation of the robot. Default is [0,0,0].
            fixedBase           :bool                   # Base of the robot fixed to the floor plane in Simulation
            viewMode            :bool                   # To view the model in the pybullet Simulation without execute the task

            IF YOU CHOOSE Real
            joints              :list[MyActuatorRMD]    # List of the robot joints - ex : [j1,j2,j3 ...]
            can_port            :int                    # Can port - ex : 0,1,2,3, ... (generaly 0 or 1)
            can_bitrate         :int                    # Can bitrate - ex : 500000 (500k), 1000000 (1M), ... (Myactuator joints setup on 1M)
        """

        self.joint_types = joint_types
        self.joint_distances = joint_distances
        self.joint_orientations = joint_orientations

        self.d = d
        self.alpha = alpha
        self.a = a
        self.theta = theta

        self.tools=Tools(joint_types=self.joint_types, joint_distances=self.joint_distances, joint_orientations=self.joint_orientations,
                             alpha=self.alpha, d=d, theta=self.theta, a=self.a)
        
        if d is None and alpha is None and a is None and theta is None: self.calculate_DHM_parameters()
        elif d is not None and alpha is not None and a is not None and theta is not None: pass
        else : print("Error : All the DHM parameters are not set !")

    # Step 1 : Calculate DHM parameters automaticly based on the specification parameters : joint_types, joint_distances, joint_orientations
    def calculate_DHM_parameters(self):
        return self.tools.calculate_DHM_parameters()
    
    # Step 2.1 : Create an URDF file based on your DHM parameters (SIMPLIFIED - No dynamics properties)
    def create_simplified_URDF_based_on_DHM_parameters(self):
        DH_Params = []
        for i in range(len(self.joint_types)):
            if self.joint_types[i]==REVOLUTE:    DH_Params.append(['r',self.d[i],self.a[i],self.alpha[i]])  
            elif self.joint_types[i]==PRISMATIC: DH_Params.append(['p',self.theta[i],self.a[i],self.alpha[i]])
        u = URDFGenerator()
        u.create_URDF(DH_Params,"outfile.urdf")

    # Step 2.2 : Create the real URDF file based on your DHM parameters thanks to the generics parts of the robot
    def create_real_URDF_based_on_DHM_parameters(self, DH_Params):
        # TODO
        pass

    # Step 3 : Launch your URDF file in simulation
    def launch_URDF_simulation(self, urdf_file="", startPos=[0,0,1], startOrientation=[0,0,0],fixedBase=False, viewMode=False):
        self.urdf_file = urdf_file
        self.simulate=Simulation(urdf_file              =urdf_file,
                                     startPos           =startPos,
                                     startOrientation   =startOrientation,
                                     fixedBase          =fixedBase,
                                     viewMode           =viewMode)
    
    # Step 4 : Setup the real robot thanks to the URDF file, but you need to specify motor models
    def setup_real_robot(self, can_port=0, can_bitrate=1000000, motor_model=MyActuatorRMD.X.V3, peak_torque=8, rated_speed=310): # 8Nm et 310rpm
        self.can_bus = CanBusGsUsb(can_port,can_bitrate)
        self.joints=[]
        for i in range(len(self.simulate.get_num_joint())): self.joints.append(motor_model(i,8,peak_torque,rated_speed,self.can_bus))
        self.can_bus.setup()

    # Step 5.1 : Move the end effector in the R0 frame
    def move_robot_position(self, X, Y, Z, end_effector_orientation, velocity_percentage, acceptable_error=1E-5, wait_to_exit=True):
        
        joint_current_position = self.simulate.get_joint_angle_current_positions()
        print("joint initial positions                  ",joint_current_position) 
        print("endEffector initial position             ",self.tools.forward_kinematics(joint_current_position))  
        print("target X,Y,Z                             ",X,Y,Z)

        # Computes models
        all_q = self.tools.inverse_kinematics([X,Y,Z,end_effector_orientation]) # Return multiples combinaison of q 
        print("target all_q (inverse kinematics)        ",all_q[0])  
        print("endEffector target position real         ",self.tools.forward_kinematics(all_q[0]))

        v = [velocity_percentage * 32.46312405 / 100] * self.simulate.get_num_joint()
        all_qdot = self.tools.Indirect_differentiel_kinematics(v,all_q[0]) # We keep the first one
        print("target all_qdot (inverse diff kinematics)",all_qdot)
        print("endEffector target speed real            ",self.tools.Direct_differential_kinematics(all_qdot, all_q[0]))

        # CAN'T WORK
        #all_torque = self.simulate.calculate_inverse_dynamics(all_q[0],all_qdot,[0.1]*self.simulate.get_num_joint())
        all_torque = [100] * self.simulate.get_num_joint()
        print("target all_torque                        ",all_torque)

        # Move all the joints in simulation
        for i in range(self.simulate.get_num_joint()):
            self.simulate.move_joint(joint_index=i, displacement=all_q[0][i], max_speed=all_qdot[i],torque=all_torque[i])

        # WARNING : IMPOSSIBLE TO SET THE TARGET SPEED
        # self.simulate.move_joints(joint_indices     =[0,1,2,3],
        #                           displacements     =[all_q[0][0],-all_q[0][1],all_q[0][2],all_q[0][3]],
        #                           max_speeds        =[all_qdot[0],all_qdot[1],all_qdot[2],all_qdot[3]], 
        #                           torques           =[100,100,100,100],
        #                           wait_to_exit      =True)

        # If you want, wait until you reach the target position
        if wait_to_exit:
            targets = all_q[0]
            while True:
                # self.simulate.update_plot_joint_positions()
                positions = [self.simulate.get_joint_angle_current_position(joint_index) for joint_index in [0, 1, 2, 3]]
                if all(abs(position - target) < acceptable_error for position, target in zip(positions, targets)):
                    break
        
        joint_current_position = self.simulate.get_joint_angle_current_positions()
        print("joint final positions                    ",joint_current_position) 
        print("endEffector final position sim           ",self.tools.forward_kinematics(joint_current_position))  
        
        # self.simulate.move_joints(joint_indices     =[0,1,2,3], 
        #                           displacements     =[all_q[0][0],all_q[0][1],all_q[0][2],all_q[0][3]], 
        #                           max_speeds        =[1,1,1,1], 
        #                           torques           =[100,100,100,100])
        print("------- END TASK --------")

    # Step 5.2 : Move the end effector in the R0 frame auto
    def move_robot_position_auto(self, X, Y, Z, max_speed, torque, acceptable_error=1E-5, wait_to_exit=True):
        
        print("joint initial positions                  ",self.simulate.get_joint_angle_current_positions()) 
        print("endEffector initial position sim         ",self.simulate.get_robot_current_position()[2])  
        print("target X,Y,Z (without offset)            ",X,Y,Z)

        # Apply offset to the target position
        endEffector_offset = self.simulate.get_endEffector_offset()
        X += endEffector_offset[0]
        Y += endEffector_offset[1]
        Z += endEffector_offset[2]
        print("endEffector offset                       ",endEffector_offset)
        print("target X,Y,Z (with offset)               ",X,Y,Z)

        # Computes models
        all_q = self.simulate.calculate_inverse_kinematics([X,Y,Z])
        print("target all_q (inverse kinematics)        ",all_q)
        all_qdot = [max_speed]*self.simulate.get_num_joint()
        all_torque = [torque]*self.simulate.get_num_joint()
        #all_torque = self.simulate.calculate_inverse_dynamics(all_q,all_qdot,[0]*self.simulate.get_num_joint())
        
        # Move all the joints in simulation
        for i in range(self.simulate.get_num_joint()):
            self.simulate.move_joint(joint_index=i, displacement=all_q[i], max_speed=all_qdot[i], torque=all_torque[i])
        
        # If you want, wait until you reach the target position
        if wait_to_exit:
            targets = all_q[0]
            while True:
                positions = [self.simulate.get_joint_angle_current_position(joint_index) for joint_index in [0, 1, 2, 3]]
                if all(abs(position - target) < acceptable_error for position, target in zip(positions, targets)):
                    break

        print("joint final positions                    ",self.simulate.get_joint_angle_current_positions()) 
        print("endEffector final position sim           ",self.simulate.get_robot_current_position()[2])  
        print("------- END TASK --------")
            
    # Step 6 : Generate a trajectory thanks to moveit
    def move_robot_moveit_trajectory(self, file_path):

        config = self.tools.extract_trajectory(file_path)
        all_q = config[0]["positions"]
        all_qdot = config[0]["velocities"]
        all_qddot = config[0]["accelerations"]
        
        print("target all_q                             ",all_q)
        print("target all_qdot                          ",all_qdot)
        print("target all_qddot                         ",all_qddot)
        
        # all_torque = self.simulate.calculate_inverse_dynamics(all_q,all_qdot,all_qddot)
        # print("target all_torque                         ",all_torque)

        # First, go to the trajectory start point
        self.simulate.move_joints(joint_indices     =[0,1,2],
                                  displacements     =all_q,
                                  max_speeds        =all_qdot,
                                  torques           =[100,100,100])
        time.sleep(1)

        # print("target X,Y,Z                             ",X,Y,Z)
        # print("all_q                                    ",all_q)
        # print("joint_current_positions                  ",self.simulate.get_joint_angle_current_position())
        # robot_data=self.simulate.get_robot_current_position_orientation()
        # print("robot_endEffector_WorldPosition          ",robot_data[0])
        # print("robot_endEffector_WorldOrientation       ",robot_data[1])
        # print("robot_endEffector_worldFramePosition     ",robot_data[2])
        # print("robot_endEffector_worldFrameOrientation  ",robot_data[3])
        
        start_time = time.time()
        
        for j in range(len(config)):
            target_time = start_time + config[j]["time_from_start"]["secs"] + config[j]["time_from_start"]["nsecs"] / 1e9
            current_time = time.time()
            if current_time < target_time:
                time.sleep(target_time - current_time)
            self.simulate.move_joints(joint_indices     =[0,1,2],
                                      displacements     =[config[j]["positions"][0],config[j]["positions"][1],config[j]["positions"][2]],
                                      max_speeds        =[config[j]["velocities"][0],config[j]["velocities"][1],config[j]["velocities"][2]], 
                                      torques           =[100,100,100])
            
            # Synchronize for the next action if needed 
            next_start_time = start_time + config[j]["time_from_start"]["secs"] + config[j]["time_from_start"]["nsecs"] / 1e9
            current_time = time.time()
            if current_time < next_start_time:
                time.sleep(next_start_time - current_time)

    def get_robot_link_frames(self):
        robot_data=self.simulate.get_robot_current_position_all_link_frames()
        for i in range(self.simulate.get_num_joint()-1):
            print(f"robot_frame{i}_WorldPosition          ",robot_data[0][i])
            print(f"robot_frame{i}_WorldOrientation       ",robot_data[1][i])
            print(f"robot_frame{i}_worldFramePosition     ",robot_data[2][i])
            print(f"robot_frame{i}_worldFrameOrientation  ",robot_data[3][i])
            print("")
        
    