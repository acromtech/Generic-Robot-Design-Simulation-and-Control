#WRAPPER MYACTUATOR
from re import S
import time
from turtle import position

#UBUNTU
import can
import os

# WINDOWS
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import (
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
)

##innomaker usb2can device do not support the GS_USB_MODE_NO_ECHO_BACK mode
from gs_usb.gs_usb import (
    GS_USB_MODE_NORMAL ,
    GS_USB_MODE_LISTEN_ONLY ,
    GS_USB_MODE_LOOP_BACK ,
    GS_USB_MODE_ONE_SHOT ,
    #GS_USB_MODE_NO_ECHO_BACK,
)

#GENERAL PARAMETERS
PORT_NAME = '/dev/ttyAMA0'
BAUDRATE = 1000000 # CAN 1Mbps

RPI=0
WINDOWS=1
OPERATING_SYSTEM = WINDOWS

devs = GsUsb.scan()
if len(devs) == 0:
    print("Can not find gs_usb device")

#set can device handle from 0 to 1,2,3...  choosing the right device serial number accroding to the print
print(devs)    
dev = devs[1]
# Close before Start device in case the device was not properly stop last time
# If do not stop the device, bitrate setting will be fail.
dev.stop()


class utils:

    def send(TX_frame):
        if(OPERATING_SYSTEM==WINDOWS):
            RX_frame=utils.send_win(TX_frame)
        elif(OPERATING_SYSTEM==RPI):
            RX_frame=utils.send_Rpi(TX_frame)
        else:
            print("Please select your operating system")
            return
        return RX_frame

    # ONLY ON UBUNTU AND RASPBIAN
    def set_can_Rpi_ON():
        #check system name, in linux will print 'posix' and in windows will print 'nt'
        print(os.name)
        os.system('sudo ifconfig can0 down')
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system("sudo ifconfig can0 txqueuelen 100000")
        os.system('sudo ifconfig can0 up')

    def send_Rpi(id,data):
        can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        msg = can.Message(id,data)
        can0.send(msg)
        time.sleep(0.001)
        print("Send frame: ",msg,"\n")

    def receive_Rpi(id):
        can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        while True:
            msg = can0.recv(10.0)
            if msg is None:
                print('No message was received')
            else:
                print(f'Received frame: \n{msg}\n')
                return msg
    
    def set_can_Rpi_OFF():
        os.system('sudo ifconfig can0 down')

    # ONLY ON WINDOWS
    def test_win():
        # Prepare frames
        set_position_0 = GsUsbFrame(can_id=0x141, data = b"\xA4\x00\xF4\x01\x00\x00\x00\x00")
        set_position_360 = GsUsbFrame(can_id=0x141, data = b"\xA4\x00\xF4\x01\xA0\x8C\x00\x00")
        temp = GsUsbFrame(can_id=0x141, data = b"\x9A\x00\x00\x00\x00\x00\x00\x00")
        frames = [
            #set_position_0
            #set_position_360
            temp
        ]

        # Read all the time and send message in each second
        end_time, n = time.time() + 1, -1
        while True:
            iframe = GsUsbFrame()
            if dev.read(iframe, 1):
                # if you don't want to receive the error frame. filter out it.
                # otherwise you will receive a lot of error frame when your device do not connet to CAN-BUS
                if iframe.can_id & CAN_ERR_FLAG != CAN_ERR_FLAG:
                    print("RX  {}".format(iframe))

            if time.time() - end_time >= 0:
                end_time = time.time() + 1
                n += 1
                n %= len(frames)

                if dev.send(frames[n]):
                    print("TX  {}".format(frames[n]))
    
    def set_can_win_ON():
        # Configuration
        if not dev.set_bitrate(BAUDRATE):
            print("Can not set bitrate for gs_usb")
            return dev
        
        # Start device, If you have only one device for test, pls use the loop-back mode,
        #dev.start(GS_USB_MODE_LOOP_BACK)
        dev.start(GS_USB_MODE_NORMAL)

    def send_win(frame):
        cpt=0
        dev.send(frame) # send message
        while True: # read all the time
            iframe = GsUsbFrame()
            if dev.read(iframe, 1):
                if iframe.can_id & CAN_ERR_FLAG != CAN_ERR_FLAG: # if you don't want to receive the error frame. 
                    if frame.can_id==iframe.can_id:
                        if frame.data[0]==iframe.data[0]:
                            cpt=cpt+1
                            if cpt==1:
                                print("TX  {}".format(frame))
                            else:
                                print("RX  {}".format(iframe))
                                return iframe
                    
    def set_can_win_OFF():
        dev.stop()
        
    def menu():
        
        print("MENU")
        print("1 : read PID parameters to RAM")
        print("2 : write PID parameters to RAM")
        print("3 : write PID parameters to ROM")
        print("4 : read acceleration")
        print("5 : write acceleration")
        print("6 : read encoder data")
        print("7 : write encoder offset")
        print("8 : write current position to ROM as motor zero command")
        print("9 : read multi-turn encoder angle")
        print("10 : read multi-turn encoder angle")
        print("11 : read motor status 1 and error flag")
        print("12 : clear motor error flag")
        print("13 : read motor status 2")
        print("14 : read motor status 3")
        print("15 : motor off command")
        print("16 : motor stop command")
        print("17 : motor running command")
        print("18 : torque closed-loop command")
        print("19 : speed closed-loop command")
        print("20 : position closed-loop command 1")
        print("21 : position closed-loop command 2")
        print("22 : position closed-loop command 3")
        print("23 : position closed-loop command 4")

        while True:
            text=input("choix : ")
            id=int(input("what's motor id : "))
            if text=="1":
                motor.read.pid_parameter_to_RAM(id)
            elif text=="2":
                current_loop_kp=int(input("what's motor current loop kp : "))
                current_loop_ki=int(input("what's motor current loop ki : "))
                speed_loop_kp=int(input("what's motor speed loop kp : "))
                speed_loop_ki=int(input("what's motor speed loop ki : "))
                position_loop_kp=int(input("what's motor position loop kp : "))
                position_loop_ki=int(input("what's motor position loop ki : "))
                motor.write.pid_parameter_to_RAM(id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki)
            elif text=="3":
                current_loop_kp=int(input("what's motor current loop kp : "))
                current_loop_ki=int(input("what's motor current loop ki : "))
                speed_loop_kp=int(input("what's motor speed loop kp : "))
                speed_loop_ki=int(input("what's motor speed loop ki : "))
                position_loop_kp=int(input("what's motor position loop kp : "))
                position_loop_ki=int(input("what's motor position loop ki : "))
                motor.write.pid_parameter_to_ROM(id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki)
            elif text=="4":
                motor.read.acceleration(id)
            elif text=="5":
                acceleration=int(input("what's motor acceleration : "))
                motor.write.acceleration_to_RAM(id,acceleration)
            elif text=="6":
                motor.read.encoder_data(id)
            elif text=="7":
                offset=int(input("what's encoder offset : "))
                motor.write.encoder_offset(id,offset)
            elif text=="8":
                motor.write.current_position_to_ROM_as_motor_zero_position(id)
            elif text=="9":
                motor.read.multiturn_encoder_position(id)
            elif text=="10":
                motor.read.singleturn_encoder_position(id)
            elif text=="11":
                motor.read.motor_status_1_error_flag(id)
            elif text=="12":
                motor.read.clear_motor_error_flag(id)
            elif text=="13":
                motor.read.motor_status_2(id)
            elif text=="14":
                motor.read.motor_status_3(id)
            elif text=="15":
                motor.shutdown(id)
            elif text=="16":
                motor.stop(id)
            elif text=="17":
                motor.running(id)
            elif text=="18":
                torque=int(input("what's motor torque : "))
                motor.write.torque_closed_loop_control(id,torque)
            elif text=="19":
                speed=int(input("what's motor speed : "))
                motor.write.speed_closed_loop_control(id,speed)
            elif text=="20":
                position=int(input("what's motor multiturn position : "))
                motor.write.position_multiturn_closed_loop_control(id,position)
            elif text=="21":
                speed=int(input("what's motor speed : "))
                position=int(input("what's motor multiturn position : "))
                motor.write.position_multiturn_speed_closed_loop_control(id,speed,position)
            elif text=="22":
                direction=int(input("what's motor direction (0=clockwise and 1=counterclockwise) : "))
                position=int(input("what's motor singleturn position : "))
                motor.write.position_singleturn_direction_closed_loop_control(id,direction,position)
            elif text=="23":
                direction=int(input("what's motor direction (0=clockwise and 1=counterclockwise) : "))
                speed=int(input("what's motor speed : "))
                position=int(input("what's motor singleturn position : "))
                motor.write.position_singleturn_speed_direction_closed_loop_control(id,speed,direction,position)
            else:
                print("error incorrect input")

            test=input("Continue ? Y/N :")
            if test=="N":
                return

    def set_multiturn_position_trapezoidal_signal(id,set_acceleration,set_speed,set_position):
        motor.write.acceleration_to_RAM(id,set_acceleration)
        motor.write.position_multiturn_speed_closed_loop_control(id,set_speed,set_position)

class motor:
    def shutdown(id):
        TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        RX_frame=utils.send(TX_frame)
        print("RX : motor shutdown")

    def stop(id):
        TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        RX_frame=utils.send(TX_frame)
        print("RX : motor stop")
    
    def running(id): #resume motor operation from motor stop command (recovery control mode before stop motor)
        TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        RX_frame=utils.send(TX_frame)
        print("RX : motor stop")


    class write:
        def pid_parameter_to_RAM(id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x31,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki])
            RX_frame=utils.send(TX_frame)
            print("RX : current loop KP set to ",RX_frame.data[2])
            print("RX : current loop KI set to ",RX_frame.data[3])
            print("RX : speed loop KP set to ",RX_frame.data[4])
            print("RX : speed loop KI set to ",RX_frame.data[5])
            print("RX : position loop KP set to ",RX_frame.data[6])
            print("RX : position loop KI set to ",RX_frame.data[7])

        def pid_parameter_to_ROM(id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x32,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki])
            RX_frame=utils.send(TX_frame)
            print("RX : current loop KP set to ",RX_frame.data[2])
            print("RX : current loop KI set to ",RX_frame.data[3])
            print("RX : speed loop KP set to ",RX_frame.data[4])
            print("RX : speed loop KI set to ",RX_frame.data[5])
            print("RX : position loop KP set to ",RX_frame.data[6])
            print("RX : position loop KI set to ",RX_frame.data[7])

        def acceleration_to_RAM(id,acceleration):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x34,0x00,0x00,0x00,(acceleration)&0xFF,((acceleration)&0xFF00)>>8,((acceleration)&0xFF0000)>>16,((acceleration)&0xFF000000)>>24])
            RX_frame=utils.send(TX_frame)
            print("RX : acceleration set to ",(RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24),"dps/s")

        def encoder_offset(id,offset):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x91,0x00,0x00,0x00,0x00,0x00,(offset)&0xFF,((offset)&0xFF00)>>8])
            RX_frame=utils.send(TX_frame)
            print("RX : encoder offset set to ",((RX_frame.data[6])+(RX_frame.data[7]<<8)))
        
        def current_position_to_ROM_as_motor_zero_position(id): # need to be powerd on again to take effet
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("RX : initial position set to ",(RX_frame[4])+(RX_frame[5]<<8)+(RX_frame[6]<<16)+(RX_frame[7]<<24))

        def torque_closed_loop_control(id,torque): # value range -2048 to 2048
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA1,0x00,0x00,0x00,(torque)&0xFF,((torque)&0xFF00)>>8,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame[1],"°C")
            print("RX : torque current value (-2048~2048):",((RX_frame[2])+(RX_frame[3]<<8)))
            print("RX : motor speed:",((RX_frame[4])+(RX_frame[5]<<8)),"dps")
            print("RX : motor position value (0~16383):",((RX_frame[6])+(RX_frame[7]<<8)))

        def speed_closed_loop_control(id,speed):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA2,0x00,0x00,0x00,(speed)&0xFF,((speed)&0xFF00)>>8,((speed)&0xFF0000)>>16,((speed)&0xFF000000)>>24])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame[1],"°C")
            print("RX : torque current value (-2048~2048):",((RX_frame[2])+(RX_frame[3]<<8)))
            print("RX : motor speed:",((RX_frame[4])+(RX_frame[5]<<8)),"dps")
            print("RX : motor position value (0~16383):",((RX_frame[6])+(RX_frame[7]<<8)))

        def position_multiturn_closed_loop_control(id,position):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA3,0x00,0x00,0x00,(position*100)&0xFF,((position*100)&0xFF00)>>8,((position*100)&0xFF0000)>>16,((position*100)&0xFF000000)>>24])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

        def position_multiturn_speed_closed_loop_control(id,speed,position):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA4,0x00,(speed)&0xFF,((speed)&0xFF00)>>8,(position*100)&0xFF,((position*100)&0xFF00)>>8,((position*100)&0xFF0000)>>16,((position*100)&0xFF000000)>>24])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")
        
        def position_singleturn_direction_closed_loop_control(id,direction,position):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA5,(direction)&0xFF,0x00,0x00,(position*100)&0xFF,((position*100)&0xFF00)>>8,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

        def position_singleturn_speed_direction_closed_loop_control(id,speed,direction,position):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0xA6,(direction)&0xFF,(speed)&0xFF,((speed)&0xFF00)>>8,(position*100)&0xFF,((position*100)&0xFF00)>>8,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

        def position_

    class read:
        def pid_parameter_to_RAM(id):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Current loop KP parameters:",RX_frame.data[2])
            print("Current loop KI parameters:",RX_frame.data[3])
            print("Speed loop KP parameters:",RX_frame.data[4])
            print("Speed loop KI parameters:",RX_frame.data[5])
            print("Position loop KP parameters:",RX_frame.data[6])
            print("Position loop KP parameters:",RX_frame.data[7])

        def acceleration(id):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x33,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Acceleration:",(RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24),"dps/s")
        
        def encoder_data(id): #multiturn_absolute
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Motor position:",((RX_frame.data[2])+(RX_frame.data[3]<<8)))
            print("Motor original position:",((RX_frame.data[4])+(RX_frame.data[5]<<8)))
            print("Motor offset:",((RX_frame.data[6])+(RX_frame.data[7]<<8)))
        
        def multiturn_encoder_position(id):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Encoder multiturn position:",((RX_frame.data[1])+(RX_frame.data[2]<<8)+(RX_frame.data[3]<<16)+(RX_frame.data[4]<<24)+(RX_frame.data[5]<<32)+(RX_frame.data[6]<<40)+(RX_frame.data[7]<<48))*0.01,"°")

        def singleturn_encoder_position(id):
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x94,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Encoder singleturn position:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")
        
        def motor_status_1_error_flag(id): # current motor temperature, voltage and error status flags
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x9A,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Voltage:",((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1,"V")
            if(RX_frame.data[7]==0x00):
                print("Voltage status: OK")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x01):
                print("Voltage status: low voltage protection")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x08):
                print("Voltage status: OK")
                print("Temperature status: over temperature protection")
            elif(RX_frame.data[7]==0x09):
                print("Voltage status: low voltage protection")
                print("Temperature status: over temperature protection")
            else:
                print("Error unknown number")

        def clear_motor_error_flag(id): # current motor temperature, voltage and error status flags
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x9B,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Voltage:",((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1,"V")
            if(RX_frame.data[7]==0x00):
                print("Voltage status: OK")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x01):
                print("Voltage status: low voltage protection")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x08):
                print("Voltage status: OK")
                print("Temperature status: over temperature protection")
            elif(RX_frame.data[7]==0x09):
                print("Voltage status: low voltage protection")
                print("Temperature status: over temperature protection")
            else:
                print("Error unknown number")
        
        def motor_status_2(id): # current motor temperature, speed and encoder position
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x9C,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8)),"A")
            print("Motor speed:",(RX_frame.data[4])+(RX_frame.data[5]<<8),"dps")
            print("Motor angle:",(RX_frame.data[6])+(RX_frame.data[7]<<8),"°")

        def motor_status_3(id): # current motor temperature, speed and encoder position
            TX_frame=GsUsbFrame(can_id=0x141+id, data=[0x9D,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=utils.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Phase A current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("Phase B current:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"A")
            print("Phase C current:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"A")

utils.set_can_win_ON()
#motor.write.position_multiturn_speed_closed_loop_control(0,500,0)
utils.set_multiturn_position_trapezoidal_signal(0,10,60,0)
utils.set_can_win_OFF()