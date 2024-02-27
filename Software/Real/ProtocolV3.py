from CanBusGsUsb import CanBusGsUsb

"""
CAN Bus
    Parameters
    - Bus interface: CAN
    - Baud rate: 1Mbps

    Message format
    - Identifier: Single motor command sending: 0x140 + ID(1~32)
    - Multi-motor command sending: 0x280
    - Reply: 0x240 + ID (1~32)
    - Frame format: data frame
    - Frame Type: Standard Frame
    - DLC: 8 bytes
"""

class ProtocolV3(CanBusGsUsb):
    # Attributes
    BASE_ADDR_ID = 0x140

    ADDR_READ_PID_PARAMETER_TO_RAM = 0x30
    ADDR_WRITE_PID_PARAMETER_TO_RAM = 0x31
    ADDR_WRITE_PID_PARAMETER_TO_ROM = 0x32

    ADDR_READ_ACCELERATION = 0x42
    ADDR_WRITE_ACCELERATION_TO_RAM = 0x43

    ADDR_READ_MULTITURN_ENCODER_POSITION = 0x60
    ADDR_READ_MULTITURN_ENCODER_ORIGINAL_POSITION = 0x61
    ADDR_READ_MULTITURN_ENCODER_ZERO_OFFSET = 0x62
    ADDR_WRITE_ENCODER_MULTITURN_VALUE_TO_ROM_AS_MOTOR_ZERO = 0x63
    ADDR_WRITE_ENCODER_CURRENT_MULTITURN_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION = 0x64

    ADDR_READ_MULTITURN_ANGLE = 0x92
    ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG = 0x9A
    ADDR_READ_MOTOR_STATUS_2 = 0x9C
    ADDR_READ_MOTOR_STATUS_3 = 0x9D

    ADDR_SHUTDOWN = 0x80
    ADDR_STOP = 0x81

    ADDR_TORQUE_CLOSED_LOOP_CONTROL = 0xA1
    ADDR_SPEED_CLOSED_LOOP_CONTROL = 0xA2
    ADDR_ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL = 0xA4
    ADDR_INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL = 0xA8

    ADDR_SYSTEM_OPERATING_MODE = 0x70
    ADDR_MOTOR_POWER_ACQUISITION = 0x71
    ADDR_SYSTEM_RESET = 0x76
    ADDR_SYSTEM_BRAKE_RELEASE = 0x77
    ADDR_SYSTEM_BRAKE_LOCK = 0x78
    
    ADDR_SYSTEM_RUNTIME_READ = 0xB1
    ADDR_SYSTEM_SOFTWARE_VERSION_DATE_READ = 0xB2
    ADDR_COMMUNICATION_INTERRUPTION_PROTECTION_TIME_SETTING = 0xB3
    ADDR_COMMUNICATION_BAUDRATE_SETTING = 0xB4

    ADDR_MULTI_MOTOR = 0x280
    ADDR_CANID_SETTING = 0x79
    ADDR_MOTION_MODE_CONTROL_COMMAND_CAN = 0x400
    ADDR_RS485ID_SETTING = 0x79

    MOTOR_STALL = 0x0002
    LOW_PRESSURE = 0x0004
    OVERVOLTAGE = 0x0008
    OVERCURRENT = 0x0010
    POWER_OVERRUN = 0x0040
    SPEEDING = 0x0100
    MOTOR_TEMP_OVER = 0x1000
    ENCODER_CALIB_ERROR = 0x2000

    def __init__(self,id:int,reducer_ratio:int,can_bus:CanBusGsUsb):
        self.id = id
        self.reducer_ratio = reducer_ratio
        self.can_bus = can_bus

    def shutdown(self, id): 
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + id, [self.ADDR_SHUTDOWN, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("RX : Motor shutdown successful" if msg else "RX : Error during motor shutdown")

    def stop(self, id):
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + id, [self.ADDR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("RX : Motor stop successful" if msg else "RX : Error during motor stop")

    def read_CANID(self):
        msg = self.can_bus.sendMessage(0x300, [self.ADDR_CANID_SETTING, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("RX : ID set successful" if msg else "RX : Error during ID set")

    def motor_status_1_error_flag(self, id):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + id, data=[self.ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print(msg)
        motor_temp = msg.data[1]
        voltage = (msg.data[3] | (msg.data[4] << 8)) * 0.1
        error_state = msg.data[6] | (msg.data[7] << 8)
        print(f"Motor temperature: {motor_temp}°C")
        print(f"Voltage: {voltage}V")
        if error_state == 0x00:
            print("Voltage status: OK")
            print("Temperature status: OK")
        else:
            if error_state & self.LOW_PRESSURE:         print("Voltage status: low voltage protection")
            if error_state & self.OVERVOLTAGE:          print("Voltage status: overvoltage protection")
            if error_state & self.OVERCURRENT:          print("Voltage status: overcurrent protection")
            if error_state & self.POWER_OVERRUN:        print("Voltage status: power overrun")
            if error_state & self.SPEEDING:             print("Voltage status: speeding")
            if error_state & self.MOTOR_TEMP_OVER:      print("Temperature status: over temperature protection")
            if error_state & self.ENCODER_CALIB_ERROR:  print("Error status: encoder calibration error")
        return motor_temp,voltage,error_state

"""
    class write:
        def pid_parameter_to_RAM(self,id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_PID_PARAMETER_TO_RAM,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki])
            RX_frame=Can.send(TX_frame)
            print("RX : current loop KP set to ",RX_frame.data[2])
            print("RX : current loop KI set to ",RX_frame.data[3])
            print("RX : speed loop KP set to ",RX_frame.data[4])
            print("RX : speed loop KI set to ",RX_frame.data[5])
            print("RX : position loop KP set to ",RX_frame.data[6])
            print("RX : position loop KI set to ",RX_frame.data[7])

        def pid_parameter_to_ROM(self,id,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_PID_PARAMETER_TO_ROM,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki])
            RX_frame=Can.send(TX_frame)
            print("RX : current loop KP set to ",RX_frame.data[2])
            print("RX : current loop KI set to ",RX_frame.data[3])
            print("RX : speed loop KP set to ",RX_frame.data[4])
            print("RX : speed loop KI set to ",RX_frame.data[5])
            print("RX : position loop KP set to ",RX_frame.data[6])
            print("RX : position loop KI set to ",RX_frame.data[7])

        def acceleration_to_RAM(self,id,acceleration):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_ACCELERATION_TO_RAM,0x00,0x00,0x00,(acceleration)&0xFF,((acceleration)&0xFF00)>>8,((acceleration)&0xFF0000)>>16,((acceleration)&0xFF000000)>>24])
            RX_frame=Can.send(TX_frame)
            print("RX : acceleration set to ",(RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24),"dps/s")

        def encoder_offset(self,id,offset):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_ENCODER_OFFSET,0x00,0x00,0x00,0x00,0x00,(offset)&0xFF,((offset)&0xFF00)>>8])
            RX_frame=Can.send(TX_frame)
            print("RX : encoder offset set to ",((RX_frame.data[6])+(RX_frame.data[7]<<8)))
        
        def current_position_to_ROM_as_motor_zero_position(self,id): # need to be powerd on again to take effet
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("RX : initial position set to ",(RX_frame[4])+(RX_frame[5]<<8)+(RX_frame[6]<<16)+(RX_frame[7]<<24))

        def torque_closed_loop_control(self,id,torque): # value range -2048 to 2048
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_TORQUE_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(torque)&0xFF,((torque)&0xFF00)>>8,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame[1],"°C")
            print("RX : torque current value (-2048~2048):",((RX_frame[2])+(RX_frame[3]<<8)))
            print("RX : motor speed:",((RX_frame[4])+(RX_frame[5]<<8)),"dps")
            print("RX : motor position value (0~16383):",((RX_frame[6])+(RX_frame[7]<<8)))

        def speed_closed_loop_control(self,id,speed):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_SPEED_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(speed)&0xFF,((speed)&0xFF00)>>8,((speed)&0xFF0000)>>16,((speed)&0xFF000000)>>24])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame[1],"°C")
            print("RX : torque current value (-2048~2048):",((RX_frame[2])+(RX_frame[3]<<8)))
            print("RX : motor speed:",((RX_frame[4])+(RX_frame[5]<<8)),"dps")
            print("RX : motor position value (0~16383):",((RX_frame[6])+(RX_frame[7]<<8)))

        def position_multiturn_closed_loop_control(self,id,position):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(position*100)&0xFF,((position*100)&0xFF00)>>8,((position*100)&0xFF0000)>>16,((position*100)&0xFF000000)>>24])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

        def position_multiturn_speed_closed_loop_control(self,id,speed,position):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL,0x00,(speed)&0xFF,((speed)&0xFF00)>>8,(position*100)&0xFF,((position*100)&0xFF00)>>8,((position*100)&0xFF0000)>>16,((position*100)&0xFF000000)>>24])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")
        
        def position_singleturn_direction_closed_loop_control(self,id,direction,position):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL,(direction)&0xFF,0x00,0x00,(position*100)&0xFF,((position*100)&0xFF00)>>8,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

        def position_singleturn_speed_direction_closed_loop_control(self,id,speed,direction,position):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL,(direction)&0xFF,(speed)&0xFF,((speed)&0xFF00)>>8,(position*100)&0xFF,((position*100)&0xFF00)>>8,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("RX : motor temperature:",RX_frame.data[1],"°C")
            print("RX : torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("RX : motor speed:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"dps")
            print("RX : motor angle:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")

    class read:
        def pid_parameter_to_RAM(self,id):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_PID_PARAMETER_TO_RAM,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Current loop KP parameters:",RX_frame.data[2])
            print("Current loop KI parameters:",RX_frame.data[3])
            print("Speed loop KP parameters:",RX_frame.data[4])
            print("Speed loop KI parameters:",RX_frame.data[5])
            print("Position loop KP parameters:",RX_frame.data[6])
            print("Position loop KP parameters:",RX_frame.data[7])

        def acceleration(self,id):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_ACCELERATION,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Acceleration:",(RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24),"dps/s")
        
        def encoder_data(self,id): #multiturn_absolute
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_ENCODER_DATA,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Motor position:",((RX_frame.data[2])+(RX_frame.data[3]<<8)))
            print("Motor original position:",((RX_frame.data[4])+(RX_frame.data[5]<<8)))
            print("Motor offset:",((RX_frame.data[6])+(RX_frame.data[7]<<8)))
        
        def multiturn_encoder_position(self,id):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_MULTITURN_ENCODER_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Encoder multiturn position:",((RX_frame.data[1])+(RX_frame.data[2]<<8)+(RX_frame.data[3]<<16)+(RX_frame.data[4]<<24)+(RX_frame.data[5]<<32)+(RX_frame.data[6]<<40)+(RX_frame.data[7]<<48))*0.01,"°")

        def singleturn_encoder_position(self,id):
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_SINGLETURN_ENCODER_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Encoder singleturn position:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"°")
        
        def motor_status_1_error_flag(self,id): # current motor temperature, voltage and error status flags
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Voltage:",((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1,"V")
            if(RX_frame.data[7]==0x00):
                print("Voltage status: OK")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x01):
                print("Voltage status: low voltage protection")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==self.NO_DLC):
                print("Voltage status: OK")
                print("Temperature status: over temperature protection")
            elif(RX_frame.data[7]==0x09):
                print("Voltage status: low voltage protection")
                print("Temperature status: over temperature protection")
            else:
                print("Error unknown number")

        def clear_motor_error_flag(self,id): # current motor temperature, voltage and error status flags
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_CLEAR_MOTOR_ERROR_FLAG,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Voltage:",((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1,"V")
            if(RX_frame.data[7]==0x00):
                print("Voltage status: OK")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==0x01):
                print("Voltage status: low voltage protection")
                print("Temperature status: OK")
            elif(RX_frame.data[7]==self.NO_DLC):
                print("Voltage status: OK")
                print("Temperature status: over temperature protection")
            elif(RX_frame.data[7]==0x09):
                print("Voltage status: low voltage protection")
                print("Temperature status: over temperature protection")
            else:
                print("Error unknown number")
        
        def motor_status_2(self,id): # current motor temperature, speed and encoder position
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_MOTOR_STATUS_2,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Torque current:",((RX_frame.data[2])+(RX_frame.data[3]<<8)),"A")
            print("Motor speed:",(RX_frame.data[4])+(RX_frame.data[5]<<8),"dps")
            print("Motor angle:",(RX_frame.data[6])+(RX_frame.data[7]<<8),"°")

        def motor_status_3(self,id): # current motor temperature, speed and encoder position
            TX_frame=GsUsbFrame(can_id=self.BASE_ADDR_ID+id, data=[self.ADDR_READ_MOTOR_STATUS_3,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
            RX_frame=Can.send(TX_frame)
            print("Motor temperature:",RX_frame.data[1],"°C")
            print("Phase A current:",((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01,"A")
            print("Phase B current:",((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01,"A")
            print("Phase C current:",((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01,"A")
"""