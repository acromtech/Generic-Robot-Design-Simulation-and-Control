import os
# Install dependencies
os.system("sudo apt install net-tools")
os.system("pip3 install python-can")
os.system("pip3 install gs-usb")
os.system("pip3 install numpy")

from Real.MyActuatorRMD import MyActuatorRMD
from Real.CanBusGsUsb import CanBusGsUsb

can_bus=CanBusGsUsb(0,1000000)
can_bus.setup()

j1 = MyActuatorRMD.L(1,1,can_bus)

j1.read_motor_status_1_error_flag()
j1.read_clear_motor_error_flag()
j1.read_motor_status_2()
j1.read_motor_status_3()

acc=j1.read_acceleration()
j1.write_acceleration_to_RAM(acc+10)
j1.read_acceleration()
j1.write_acceleration_to_RAM(acc)
j1.read_acceleration()

j1.read_pid_parameter_to_RAM()
j1.read_encoder_data()
j1.read_multiturn_encoder_position()
j1.read_singleturn_encoder_position()

can_bus.shutdown()