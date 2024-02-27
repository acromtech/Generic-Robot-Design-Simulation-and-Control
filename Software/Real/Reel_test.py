import os
# Install dependencies
#os.system("pip3 install python-can")
#os.system("pip3 install gs-usb")
#os.system("pip3 install numpy")
import time

from MyActuatorRMD import MyActuatorRMD
from CanBusGsUsb import CanBusGsUsb

can_bus=CanBusGsUsb(0,1000000)
can_bus.setup()

j1 = MyActuatorRMD.L.V1(1,1,can_bus)

##########################################
# TESTS UNITAIRES
##########################################

print("\n\nTEST : PID PARAMETERS")
kp1,ki1,kp2,ki2,kp3,ki3=j1.write_pid_parameter_to_RAM(110,80,5,5,20,50) 
print(kp1,ki1,kp2,ki2,kp3,ki3)
if kp1==110 and ki1==80 and kp2==5 and ki2==5 and kp3==20 and ki3==50:print("PID PARAMETERS : OK")
else:print("PID PARAMETERS : NOK")
kp1,ki1,kp2,ki2,kp3,ki3=j1.write_pid_parameter_to_RAM(100,100,40,14,30,30) 


print("\n\nTEST : MOTOR ACCELERATION")
j1.read_acceleration()
if j1.write_acceleration_to_RAM(200)==200:print("ACCELERATION : OK")
else:print("ACCELERATION : NOK")


print("\n\nTEST : MOTOR STATUS 1")
motor_temp,voltage,error_state=j1.read_motor_status_1_error_flag()
if motor_temp<50 and voltage>20 and voltage<26 and error_state==0x00 :print("MOTOR STATUS 1 : OK")
else:print("MOTOR STATUS 1 : NOK")


print("\n\nTEST : MOTOR STATUS 2")
motor_temp, torque_current, motor_speed, encoder_position=j1.read_motor_status_2()
if motor_temp<50 and torque_current>-2048 and torque_current<2048 and encoder_position>0 and encoder_position<65535:print("MOTOR STATUS 2 : OK")
else:print("MOTOR STATUS 2 : NOK")


print("\n\nTEST : MOTOR STATUS 3")
motor_temp, phase_A_current, phase_B_current, phase_C_current=j1.read_motor_status_3()
if motor_temp<50 :print("MOTOR STATUS 3 : OK")
else:print("MOTOR STATUS 3 : NOK")


print("\n\nTEST : SPEED CLOSED LOOP CONTROL")
j1.speed_closed_loop_control(360)	#360°/s soit 1tr/s
time.sleep(3)
if j1.stop() : print("STOP : OK")
else : print("STOP : NOK")
time.sleep(1)
if j1.running() : print("RUNNING \t OK") 
else : print("RUNNING \t NOK")
time.sleep(3)
j1.motor_off()
time.sleep(1)

#print("\n\nTEST : TORQUE CLOSED LOOP CONTROL")
#j1.torque_closed_loop_control(0.2) # Essayer de freiner le moteur gentilement
#time.sleep(10)
#j1.torque_closed_loop_control(0.1) # Essayer de freiner le moteur gentilement
#time.sleep(10)
#j1.torque_closed_loop_control(0.5) # Essayer de freiner le moteur gentilement
#time.sleep(10)

j1.write_current_position_to_ROM_as_motor_zero_position()

print("\n\nTEST : MULTITURN SPEED CLOSED LOOP CONTROL")
j1.position_closed_loop_control_2(360,0)#360°/s,0°
j1.position_closed_loop_control_2(360,720)#360°/s,720°
time.sleep(2)

print("\n\nTEST : SINGLETURN SPEED DIRECTION CLOSED LOOP CONTROL")
j1.position_closed_loop_control_4(360,1,30)#360°/s,anticlockwise,30°
j1.position_closed_loop_control_4(360,0,180)#360°/s,clockwise,180°
j1.position_closed_loop_control_4(360,1,0)#360°/s,anticlockwise,0°
time.sleep(2)

print("\n\nTEST : SINGLETURN DIRECTION CLOSED LOOP CONTROL")
j1.position_closed_loop_control_3(1,30)
j1.position_closed_loop_control_3(1,180)
j1.position_closed_loop_control_3(1,0)
time.sleep(2)

print("\n\nTEST : MULTITURN CLOSED LOOP CONTROL")
j1.position_closed_loop_control_1(30)
j1.position_closed_loop_control_1(180)
j1.position_closed_loop_control_1(0)
time.sleep(2)

print("\n\nTEST : ENCODER DATA")
j1.read_encoder_data(debug=True)
j1.position_closed_loop_control_1(30)
print()
j1.read_encoder_data(debug=True)
j1.position_closed_loop_control_1(180)
print()
j1.read_encoder_data(debug=True)
j1.position_closed_loop_control_1(0)


print()
can_bus.shutdown()