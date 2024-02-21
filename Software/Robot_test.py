import numpy
import time
from Robot import Robot

# USE CASE : Robotic_Design
# ============================================================
robot_3DoF = Robot (joint_type              =[1,1,1],       # Optionnal (potentially for Etienne-Dombre Modelisation) -> for generic use cases
                    joint_distance          =[10,20,15],    
                    joint_orientation       =[0,0,1],       # Optionnal (potentially for Etienne-Dombre Modelisation) -> for generic use cases
                    q_config                =[0,0,0],
                    test_type               ="Robotic_Design"
                    )
#To call a specific method -> robot_3DoF.tools.method()

# USE CASE : Simulation
# ============================================================
robot_URDF = Robot (URDF_file               ="kuka_iiwa/model.urdf",
                    test_type               ="Simulation"
                    )
# To call a specific method -> robot_URDF.simulate.method()

# USE CASE : Real -> Use only if you have the USB-CAN Transceiver plugged
# CURRENTLY THIS PART ISN'T READY TO USE
# ============================================================
# can_bus=CanBusGsUsb(0,1000000)
# can_bus.setup()

# j1 = MyActuatorRMD.L(1,1,can_bus)
# j2 = MyActuatorRMD.L(2,1,can_bus)
# j3 = MyActuatorRMD.L(3,1,can_bus)

# robot_3DoF = Robot (joints                  =[j1,j2,j3],
#                     q_config                =[0,0,0], 
#                     mechanical_features     =[10,20,15],  
#                     test_type               ="Real", 
#                     )