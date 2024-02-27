from CanBusGsUsb import CanBusGsUsb

class ProtocolV2(CanBusGsUsb):
    # Attributes
    BASE_ADDR_ID = 0x140

    ADDR_READ_POSITION_LOOP_KP = 0x30
    ADDR_READ_POSITION_LOOP_KI = 0x31
    ADDR_READ_SPEED_LOOP_KP = 0x32
    ADDR_READ_SPEED_LOOP_KI = 0x33
    ADDR_READ_CURRENT_LOOP_KP = 0x34
    ADDR_READ_CURRENT_LOOP_KI = 0x35

    ADDR_WRITE_POSITION_LOOP_KP_TO_RAM = 0x36
    ADDR_WRITE_POSITION_LOOP_KI_TO_RAM = 0x37
    ADDR_WRITE_SPEED_LOOP_KP_TO_RAM = 0x38
    ADDR_WRITE_SPEED_LOOP_KI_TO_RAM = 0x39
    ADDR_WRITE_CURRENT_LOOP_KP_TO_RAM = 0x3A
    ADDR_WRITE_CURRENT_LOOP_KI_TO_RAM = 0x3B

    ADDR_WRITE_POSITION_LOOP_KP_TO_ROM = 0x3C
    ADDR_WRITE_POSITION_LOOP_KI_TO_ROM = 0x3D
    ADDR_WRITE_SPEED_LOOP_KP_TO_ROM = 0x3E
    ADDR_WRITE_SPEED_LOOP_KI_TO_ROM = 0x3F
    ADDR_WRITE_CURRENT_LOOP_KP_TO_ROM = 0x40
    ADDR_WRITE_CURRENT_LOOP_KI_TO_ROM = 0x41

    ADDR_READ_ACCELERATION = 0x42
    ADDR_WRITE_ACCELERATION_TO_RAM = 0x43

    ADDR_READ_MULTITURN_ENCODER_POSITION = 0x60
    ADDR_READ_MULTITURN_ENCODER_ORIGINAL_POSITION = 0x61
    ADDR_READ_MULTITURN_ENCODER_OFFSET = 0x62
    ADDR_WRITE_MULTITURN_ENCODER_VALUES_TO_ROM_AS_MOTOR_ZERO = 0x63
    ADDR_WRITE_MULTITURN_ENCODER_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO = 0x64
    
    ADDR_READ_SINGLETURN_ENCODER_DATA = 0x90
    ADDR_WRITE_ENCODER_VALUES_TO_ROM_AS_MOTOR_ZERO = 0x91
    ADDR_WRITE_SINGLETURN_ENCODER_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO = 0x19
    ADDR_READ_MULTITURN_ANGLE = 0x92
    ADDR_READ_SINGLE_CIRCLE_ANGLE = 0x94

    ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG = 0x9A
    ADDR_READ_MOTOR_STATUS_2 = 0x9C
    ADDR_READ_MOTOR_STATUS_3 = 0x9D

    ADDR_MOTOR_OFF = 0x80
    ADDR_STOP = 0x81
    ADDR_RUNNING = 0x88

    ADDR_TORQUE_CLOSED_LOOP_CONTROL = 0xA1
    ADDR_SPEED_CLOSED_LOOP_CONTROL = 0xA2
    ADDR_POSITION_MULTITURN_CLOSED_LOOP_CONTROL = 0xA3
    ADDR_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL = 0xA4
    ADDR_POSITION_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL = 0xA5
    ADDR_POSITION_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL = 0xA6
    ADDR_MULTITURN_INCREMENTAL_POSITION_CONTROL = 0xA7  # A VERIFER
    ADDR_MULTITURN_INCREMENTAL_POSITION_CONTROL = 0xA8 # A VERIFER

    ADDR_READ_RUNNING_MODE = 0x70
    ADDR_READ_POWER_VALUE = 0x71
    ADDR_READ_BATTERY_VOLTAGE = 0x72
    ADDR_TF = 0x73
    ADDR_SYSTEM_RESET = 0x76
    ADDR_BRAKE_OPENING = 0x77
    ADDR_BRAKE_CLOSE = 0x78
    ADDR_CAN_ID_SETTING_AND_READING = 0x79

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


    def read_position_loop_Kp(self,debug=None):
        """
        The host sends the command to read the current Position loop KP parameters.

        Data field Description Data
        DATA[0] command byte 0x30
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply(one frame)
        The drive reply data contains the Kp parameter of the position loop, which is converted usingtheQformat (Q24)
        eg,kp=0.25,Position loop after conversion kp, anglePidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x30
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKp)
        DATA[5] Position loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKp)+1)
        DATA[6] Position loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKp)+2)
        DATA[7] Position loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKp)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_POSITION_LOOP_KP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        position_loop_KP = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Position loop KP parameters: {position_loop_KP}")
        return position_loop_KP
    
    def read_position_loop_Ki(self,debug=None):
        """
        The host sends the command to read the current Position loop Ki parameters. 
        
        Data field Description Data
        DATA[0] command byte 0x31
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply(one frame)
        The drive reply data contains the Ki parameter of the position loop,which is converted using the Q format (Q24)
        eg, ki=0.25,Position loop after conversion ki,anglePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x31
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKi)
        DATA[5] Position loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKi)+1)
        DATA[6] Position loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKi)+2)
        DATA[7] Position loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKi)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_POSITION_LOOP_KI, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        position_loop_KI = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Position loop KI parameters: {position_loop_KI}")
        return position_loop_KI
    
    def read_speed_loop_Kp(self,debug=None):
        """
        The host sends the command to read the current Speed loop KP parameters. 
        
        Data field Description Data
        DATA[0] command byte 0x32
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply(one frame)
        The drive reply data contains the Kp parameter of the speed loop,which is converted using the Q format (Q24)
        eg, kp=0.25,speed loop after conversion kp,speedPidKp=0.25*16777216=4194304;
        
        Data field Description Data
        DATA[0] command byte 0x32
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKp)
        DATA[5] speed loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKp)+1)
        DATA[6] speed loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKp)+2)
        DATA[7] speed loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKp)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_SPEED_LOOP_KP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        speed_loop_KP = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Speed loop KP parameters: {speed_loop_KP}")
        return speed_loop_KP
    
    def read_speed_loop_Ki(self,debug=None):
        """
        The host sends the command to read the current Speed loop Ki parameters. 
        
        Data field Description Data
        DATA[0] command byte 0x33
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply(one frame)
        The drive reply data contains the Ki parameter of the speed loop,which is converted using the Q format (Q24)
        eg, ki=0.25,speed loop after conversion ki,speedPidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x33
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKi)
        DATA[5] speed loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKi)+1)
        DATA[6] speed loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKi)+2)
        DATA[7] speed loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKi)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_SPEED_LOOP_KI, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        speed_loop_KI = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Speed loop KI parameters: {speed_loop_KI}")
        return speed_loop_KI
    
    def read_current_loop_Kp(self,debug=None):
        """
        The host sends the command to read the Current loop Kp parameters. 
        
        Data field Description Data
        DATA[0] command byte 0x34
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00
        
        Drive reply(one frame)
        The drive reply data contains the Kp parameter of the current loop,which is converted using the Q format (Q24)
        eg, kp=0.25,current loop after conversion kp,torquePidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x34
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKp)
        DATA[5] current loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKp)+1)
        DATA[6] current loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKp)+2)
        DATA[7] current loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKp)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_CURRENT_LOOP_KP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        current_loop_KP = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Current loop KP parameters: {current_loop_KP}")
        return current_loop_KP
    
    def read_current_loop_Ki(self,debug=None):
        """
        The host sends the command to read the current Current loop Ki parameters. 
        
        Data field Description Data
        DATA[0] command byte 0x35
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply(one frame)
        The drive reply data contains the Ki parameter of the current loop,which is converted using the Q format (Q24)
        eg, ki=0.25,current loop after conversion ki,torquePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x35
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKi)
        DATA[5] current loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKi)+1)
        DATA[6] current loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKi)+2)
        DATA[7] current loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKi)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_CURRENT_LOOP_KI, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        current_loop_KI = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print(f"Current loop KI parameters: {current_loop_KI}")
        return current_loop_KI


    def write_position_loop_Kp_to_RAM(self,position_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of position loop to the RAM, andthewriteparameters are invalid after the power off,and the Q format (Q24) is used for conversion. eg，Kp=0.25,position loop after conversion Kp,anglePidKp=0.25*16777216=4194304;
        
        Data field Description Data
        DATA[0] command byte 0x36
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKp)
        DATA[5] Position loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKp)+1)
        DATA[6] Position loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKp)+2)
        DATA[7] Position loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKp)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the reply command is the sameasthereceived command.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_POSITION_LOOP_KP_TO_RAM, 0x00, 0x00, 0x00, (position_loop_kp >> 0) & 0xFF,(position_loop_kp >> 8) & 0xFF,(position_loop_kp >> 16) & 0xFF,(position_loop_kp >> 24) & 0xFF])
        position_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Position loop Kp set to ", position_loop_kp)
        return position_loop_kp
    
    def write_position_loop_Ki_to_RAM(self,position_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of position loop to the RAM, and the write parameters are invalid after the power off,and the Q format (Q24) is used for conversion. 
        eg, Ki=0.25,position loop after conversion Ki,anglePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x37
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKi)
        DATA[5] Position loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKi)+1)
        DATA[6] Position loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKi)+2)
        DATA[7] Position loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_POSITION_LOOP_KI_TO_RAM, 0x00, 0x00, 0x00, (position_loop_ki >> 0) & 0xFF,(position_loop_ki >> 8) & 0xFF,(position_loop_ki >> 16) & 0xFF,(position_loop_ki >> 24) & 0xFF])
        position_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Position loop Ki set to ", position_loop_ki)
        return position_loop_ki
    
    def write_speed_loop_Kp_to_RAM(self,speed_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of speed loop to the RAM, and the write parameters are invalid after the power off,and the Q format (Q24) is used for conversion. 
        eg, Kp=0.25,speed loop after conversion kp,speedPidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x38
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKp)
        DATA[5] speed loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKp)+1)
        DATA[6] speed loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKp)+2)
        DATA[7] speed loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKp)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_SPEED_LOOP_KP_TO_RAM, 0x00, 0x00, 0x00, (speed_loop_kp >> 0) & 0xFF,(speed_loop_kp >> 8) & 0xFF,(speed_loop_kp >> 16) & 0xFF,(speed_loop_kp >> 24) & 0xFF])
        speed_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Speed loop Kp set to ", speed_loop_kp)
        return speed_loop_kp
    
    def write_speed_loop_Ki_to_RAM(self,speed_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of speed loop to the RAM, and the write parameters are invalid after the power is turned off,and the Q format (Q24) is used for conversion. 
        eg, Ki=0.25,speed loop after conversion Ki,speedPidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x39
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKi)
        DATA[5] speed loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKi)+1)
        DATA[6] speed loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKi)+2)
        DATA[7] speed loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_SPEED_LOOP_KI_TO_RAM, 0x00, 0x00, 0x00, (speed_loop_ki >> 0) & 0xFF,(speed_loop_ki >> 8) & 0xFF,(speed_loop_ki >> 16) & 0xFF,(speed_loop_ki >> 24) & 0xFF])
        speed_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Speed loop Ki set to ", speed_loop_ki)
        return speed_loop_ki
    
    def write_current_loop_Kp_to_RAM(self,current_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of current loop to the RAM, and write parameters are invalid after the power off,and the Q format (Q24) is used for conversion. 
        eg, kp=0.25,current loop after conversion kp,torquePidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3A
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKp)
        DATA[5] current loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKp)+1)
        DATA[6] current loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKp)+2)
        DATA[7] current loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKp)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_CURRENT_LOOP_KP_TO_RAM, 0x00, 0x00, 0x00, (current_loop_kp >> 0) & 0xFF,(current_loop_kp >> 8) & 0xFF,(current_loop_kp >> 16) & 0xFF,(current_loop_kp >> 24) & 0xFF])
        current_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Current loop Kp set to ", current_loop_kp)
        return current_loop_kp
    
    def write_current_loop_Ki_to_RAM(self,current_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of current loop to the RAM, and the write parameters are invalid after the power is turned off,and the Q format (Q24) is used for conversion. 
        eg, Ki=0.25,current loop after conversion Ki,torquePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3B
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKi)
        DATA[5] current loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKi)+1)
        DATA[6] current loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKi)+2)
        DATA[7] current loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_CURRENT_LOOP_KI_TO_RAM, 0x00, 0x00, 0x00, (current_loop_ki >> 0) & 0xFF,(current_loop_ki >> 8) & 0xFF,(current_loop_ki >> 16) & 0xFF,(current_loop_ki >> 24) & 0xFF])
        current_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Current loop Ki set to ", current_loop_ki)
        return current_loop_ki


    def write_position_loop_Kp_to_ROM(self,position_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of position loop to the ROM, and the write parameters are valid after the power off,and the Q format (Q24) is used for conversion. 
        eg, kp=0.25,position loop after conversion kp,anglePidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3C
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] position loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKp)
        DATA[5] position loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKp)+1)
        DATA[6] position loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKp)+2)
        DATA[7] position loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKp)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_POSITION_LOOP_KP_TO_ROM, 0x00, 0x00, 0x00, (position_loop_kp >> 0) & 0xFF,(position_loop_kp >> 8) & 0xFF,(position_loop_kp >> 16) & 0xFF,(position_loop_kp >> 24) & 0xFF])
        position_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Position loop Kp set to ", position_loop_kp)
        return position_loop_kp
    
    def write_position_loop_Ki_to_ROM(self,position_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of position loop to the ROM, and the write parameters are valid after the power off,and the Q format (Q24) is used for conversion. 
        eg, ki=0.25,position loop after conversion Ki,anglePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3D
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] position loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&anglePidKi)
        DATA[5] position loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&anglePidKi)+1)
        DATA[6] position loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&anglePidKi)+2)
        DATA[7] position loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&anglePidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_POSITION_LOOP_KI_TO_ROM, 0x00, 0x00, 0x00, (position_loop_ki >> 0) & 0xFF,(position_loop_ki >> 8) & 0xFF,(position_loop_ki >> 16) & 0xFF,(position_loop_ki >> 24) & 0xFF])
        position_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Position loop Ki set to ", position_loop_ki)
        return position_loop_ki
    
    def write_speed_loop_Kp_to_ROM(self,speed_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of speed loop to the ROM, and the write parameters are valid after the power off,and the Q format (Q24) is used for conversion. 
        eg, kp=0.25,speed loop after conversion kp,speedPidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3E
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKp)
        DATA[5] speed loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKp)+1)
        DATA[6] speed loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKp)+2)
        DATA[7] speed loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKp)+3)
        
        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_SPEED_LOOP_KP_TO_ROM, 0x00, 0x00, 0x00, (speed_loop_kp >> 0) & 0xFF,(speed_loop_kp >> 8) & 0xFF,(speed_loop_kp >> 16) & 0xFF,(speed_loop_kp >> 24) & 0xFF])
        speed_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Speed loop Kp set to ", speed_loop_kp)
        return speed_loop_kp
    
    def write_speed_loop_Ki_to_ROM(self,speed_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of speed loop to the ROM, and the write parameters are valid after the power off,and the Q format (Q24) is used for conversion. 
        eg, ki=0.25,speed loop after conversion Ki,speedPidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x3F
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] speed loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&speedPidKi)
        DATA[5] speed loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&speedPidKi)+1)
        DATA[6] speed loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&speedPidKi)+2)
        DATA[7] speed loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&speedPidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_SPEED_LOOP_KI_TO_ROM, 0x00, 0x00, 0x00, (speed_loop_ki >> 0) & 0xFF,(speed_loop_ki >> 8) & 0xFF,(speed_loop_ki >> 16) & 0xFF,(speed_loop_ki >> 24) & 0xFF])
        speed_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Speed loop Ki set to ", speed_loop_ki)
        return speed_loop_ki
    
    def write_current_loop_Kp_to_ROM(self,current_loop_kp,debug=None):
        """
        The host sends the command to write the Kp parameters of current loop to the ROM, and the write parameters are valid after the power off,and the Q format (Q24) is used for conversion. 
        eg, kp=0.25,current loop after conversion kp,torquePidKp=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] NULL 0x40
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Kp parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKp)
        DATA[5] current loop Kp parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKp)+1)
        DATA[6] current loop Kp parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKp)+2)
        DATA[7] current loop Kp parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKp)+3)
        
        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_CURRENT_LOOP_KP_TO_ROM, 0x00, 0x00, 0x00, (current_loop_kp >> 0) & 0xFF,(current_loop_kp >> 8) & 0xFF,(current_loop_kp >> 16) & 0xFF,(current_loop_kp >> 24) & 0xFF])
        current_loop_kp = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Current loop Kp set to ", current_loop_kp)
        return current_loop_kp
    
    def write_current_loop_Ki_to_ROM(self,current_loop_ki,debug=None):
        """
        The host sends the command to write the Ki parameters of current loop to the ROM, and the write parameters are valid after the power is turned off,and the Q format (Q24) is used for conversion. 
        eg, ki=0.25,current loop after conversion Ki ,torquePidKi=0.25*16777216=4194304;

        Data field Description Data
        DATA[0] command byte 0x41
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] current loop Ki parameters low byte 1 DATA[4] = *(uint8_t *)(&torquePidKi)
        DATA[5] current loop Ki parameters byte 2 DATA[5] = *((uint8_t *)(&torquePidKi)+1)
        DATA[6] current loop Ki parameters byte 3 DATA[6] = *((uint8_t *)(&torquePidKi)+2)
        DATA[7] current loop Ki parameters byte 4 DATA[7] = *((uint8_t *)(&torquePidKi)+3)

        Drive reply(one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_CURRENT_LOOP_KI_TO_ROM, 0x00, 0x00, 0x00, (current_loop_ki >> 0) & 0xFF,(current_loop_ki >> 8) & 0xFF,(current_loop_ki >> 16) & 0xFF,(current_loop_ki >> 24) & 0xFF])
        current_loop_ki = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("Current loop Ki set to ", current_loop_ki)
        return current_loop_ki
	

    def read_acceleration(self,debug=None):
        """
        The host send the command to read motor acceleration data

        Data field Description Data
        DATA[0] command byte 0x42
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The driver reply data include acceleration data, data type :int32_t, unit:1dps/s,Parameter range0-10000. 
        
        Data field Description Data
        DATA[0] command byte 0x42
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Acceleration low byte 1 DATA[4] = *(uint8_t *)(&Accel)
        DATA[5] Acceleration byte 2 DATA[5] = *((uint8_t *)(&Accel)+1)
        DATA[6] Acceleration byte 3 DATA[6] = *((uint8_t *)(&Accel)+2)
        DATA[7] Acceleration byte 4 DATA[7] = *((uint8_t *)(&Accel)+3)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_ACCELERATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        acceleration = (msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24))
        if debug != None: print(f"Acceleration (range 0-10000) : {acceleration} dps/s")
        return acceleration
	
    def write_acceleration_to_RAM(self, acceleration,debug=None):
        """
        The host sends the command to write the acceleration to the RAM, and the write parameters are invalid after the power is turned off. 
        Acceleration data Accel is int32_t type, unit 1dps/s,Parameter range0-10000. 
        
        Data field Description Data
        DATA[0] command byte 0x43
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Acceleration low byte 1 DATA[4] = *(uint8_t *)(&Accel)
        DATA[5] Acceleration byte 2 DATA[5] = *((uint8_t *)(&Accel)+1)
        DATA[6] Acceleration byte 3 DATA[6] = *((uint8_t *)(&Accel)+2)
        DATA[7] Acceleration byte 4 DATA[7] = *((uint8_t *)(&Accel)+3)

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_ACCELERATION_TO_RAM, 0x00, 0x00, 0x00, (acceleration >> 0) & 0xFF,(acceleration >> 8) & 0xFF,(acceleration >> 16) & 0xFF,(acceleration >> 24) & 0xFF])
        acceleration = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        if debug != None: print("RX : Acceleration set to ", acceleration)
        return acceleration
    
    def read_multiturn_encoder_position(self,debug=None):
        """
        The host sends this command to read the encoder multi-turn position. 
        
        Data field Description Data
        DATA[0] command byte 0x60
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command. 
        The frame data contains the following parameters. 
        Encoder multiturn position (int64_t type, multiturn encoder value range ,valid data is 6 bytes),whichistheencoder original position minus the encoder multiturn zero offset value,the seventh byte representspositive and negative, 0 is positive, and 1 is negative.
        
        Data field Description Data
        DATA[0] command byte 0x60
        DATA[1] Encoder position low byte1 DATA[0] = *(uint8_t *)(&encoder)
        DATA[2] Encoder position byte2 DATA[1] = *((uint8_t *)(&encoder)+1)
        DATA[3] Encoder position byte3 DATA[2] = *((uint8_t *)(&encoder)+2)
        DATA[4] Encoder position byte4 DATA[3] = *((uint8_t *)(&encoder)+3)
        DATA[5] Encoder position byte5 DATA[4] = *((uint8_t *)(&encoder)+4)
        DATA[6] Encoder position byte6 DATA[5] = *((uint8_t *)(&encoder)+5)
        DATA[7] Encoder position positive or negative flag bit 1 minus , 0 plus
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MULTITURN_ENCODER_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_position = msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40)
        if msg.data[7] != 0: encoder_position = -encoder_position
        if debug is not None: print(f"Multiturn encoder position: {encoder_position}")
        return encoder_position

    def read_multiturn_encoder_original_position(self,debug=None):
        """
        The host sends this command to read the encoder multi-turn position. 
        
        Data field Description Data
        DATA[0] command byte 0x61
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command. 
        The frame data contains the following parameters. 
        Encoder multiturn original position encoderRaw(int64_t type, value range ,valid data is 6 bytes),theseventh byte represents positive and negative, 0 is positive, and 1 is negative. 
        
        Data field Description Data
        DATA[0] command byte 0x61
        DATA[1] Encoder original position byte1 DATA[0] = *(uint8_t *)(&encoderRaw)
        DATA[2] Encoder original position byte2 DATA[1] = *((uint8_t *)(&encoderRaw)+1)
        DATA[3] Encoder original position byte3 DATA[2] = *((uint8_t *)(&encoderRaw)+2)
        DATA[4] Encoder original position byte4 DATA[3] = *((uint8_t *)(&encoderRaw)+3)
        DATA[5] Encoder original position byte5 DATA[4] = *((uint8_t *)(&encoderRaw)+4)
        DATA[6] Encoder original position byte6 DATA[5] = *((uint8_t *)(&encoderRaw)+5)
        DATA[7] Encoder original position positive or negative flag bit 1 negative, 0 positive
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MULTITURN_ENCODER_ORIGINAL_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_raw = msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40)
        if msg.data[7] != 0: encoder_raw = -encoder_raw
        if debug is not None: print(f"Multiturn encoder original position : {encoder_raw}")
        return encoder_raw
    
    def read_multiturn_encoder_zero_offset(self,debug=None):
        """
        The host sends this command to read the encoder multi-turn zero offset value. 
        
        Data field Description Data
        DATA[0] command byte 0x62
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command. The frame data contains the followingparameters. encoder multi-turn zero offset encoderOffset（int64_t type,value range,valid data is 6 bytes）,theseventhbyte represents positive and negative, 0 is positive, and 1 is negative. 
        
        Data field Description Data
        DATA[0] command byte 0x62
        DATA[1] Encoder offset byte1 DATA[0] = *(uint8_t *)(&encoderOffset)
        DATA[2] Encoder offset byte2 DATA[1] = *((uint8_t *)(&encoderOffset)+1)
        DATA[3] Encoder offset byte3 DATA[2] = *((uint8_t *)(&encoderOffset)+2)
        DATA[4] Encoder offset byte4 DATA[3] = *((uint8_t *)(&encoderOffset)+3)
        DATA[5] Encoder offset byte5 DATA[4] = *((uint8_t *)(&encoderOffset)+4)
        DATA[6] Encoder offset byte6 DATA[5] = *((uint8_t *)(&encoderOffset)+5)
        DATA[7] Encoder offset positive or negative flag bit 1 negative , 0 positive
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MULTITURN_ENCODER_OFFSET, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_zero_offset = msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40)
        if msg.data[7] != 0 : encoder_zero_offset = -encoder_zero_offset
        if debug != None: print(f"Multiturn encoder zero offset : {encoder_zero_offset}")
        return encoder_zero_offset
    
    def write_multiturn_encoder_value_position_to_ROM_as_motor_zero(self,debug=None):
        """
        The host sends this command to set the encoder zero offset, where the encoder multi-turn valueencoder Offset that needs to be written is int64_t type,(value range,valid data is 6 bytes)，theseventhbyte represents positive and negative, 0 is positive, and 1 is negative. 
        
        Data field Description Data
        DATA[0] command byte 0x63
        DATA[1] Encoder offset low byte1 DATA[0] = *(uint8_t *)(&encoderOffset)
        DATA[2] Encoder offset byte2 DATA[1] = *((uint8_t *)(&encoderOffset)+1)
        DATA[3] Encoder offset byte3 DATA[2] = *((uint8_t *)(&encoderOffset)+2)
        DATA[4] Encoder offset byte4 DATA[3] = *((uint8_t *)(&encoderOffset)+3)
        DATA[5] Encoder offset byte5 DATA[4] = *((uint8_t *)(&encoderOffset)+4)
        DATA[6] Encoder offset byte6 DATA[5] = *((uint8_t *)(&encoderOffset)+5)
        DATA[7] Encoder offset positive or negative flag bit 1 negative , 0 positive

        Drive reply (one frame)
        The motor responds to the host after receiving the command. The frame data contains the following parameters
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_MULTITURN_ENCODER_VALUES_TO_ROM_AS_MOTOR_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_offset = msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40)
        if msg.data[7] != 0 : encoder_offset = -encoder_offset
        if debug != None: print(f"Multiturn encoder offset set to : {encoder_offset}")
        return encoder_offset

    def write_multiturn_encoder_current_position_to_ROM_as_motor_zero(self,debug=None):
        """
        Write the current encoder position of the motor as the initial position to the ROM

        Notice:
        1.This command needs to be re-powered to take effect
        2.This command will write the zero position to the ROM of the drive. Multiple writes will affect thelifeofthe chip. Frequent use is not recommended. 
        
        Data field Description Data
        DATA[0] command byte 0x64
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor reply to the host after receiving the command, and the data of encode roffset is the 0 offset value.

        Data field Description Data
        DATA[0] command byte 0x64
        DATA[1] encoder offset low byte1 DATA[0] = *(uint8_t *)(&encoderOffset)
        DATA[2] encoder offset byte2 DATA[1] = *((uint8_t *)(&encoderOffset)+1)
        DATA[3] encoder offset byte3 DATA[2] = *((uint8_t *)(&encoderOffset)+2)
        DATA[4] encoder offset byte4 DATA[3] = *((uint8_t *)(&encoderOffset)+3)
        DATA[5] encoder offset byte5 DATA[4] = *((uint8_t *)(&encoderOffset)+4)
        DATA[6] encoder offset byte6 DATA[5] = *((uint8_t *)(&encoderOffset)+5)
        DATA[7] Encoder offset positive or negative flag bit 1 negative , 0 positive 
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_MULTITURN_ENCODER_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_offset = (msg.data[4] | (msg.data[1] << 8) | (msg.data[2] << 16) | (msg.data[3] << 24) | (msg.data[4] << 32) | (msg.data[5] << 40) | (msg.data[6] << 48))
        if msg.data[7] != 0 : encoder_offset = -encoder_offset
        if debug != None: print(f"Multiturn encoder offset set to : {encoder_offset}")
        return encoder_offset

    def read_singleturn_encoder_data(self,debug=None):
        """
        The host sends this command to read the current position of the encoder. 
        Note that thecurrentcommand is used as a single-turn data reading command for direct drive motors. 
        
        Data field Description Data
        DATA[0] command byte 0x90
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00
        
        Drive reply (one frame)
        The motor responds to the host after receiving the command.
        The frame data contains the following parameters. 
        1.Encoder position encoder (int16_t type, the value range of 16bit encoder is 0~65535), which is the value of the original position of the encoder minus the zero offset of the encoder.
        2.Encoder's original position encoderRaw (uint16_t type, the value range of 16bit encoder is 0~65535). 3.Encoder offset encoderOffset (uint16_t type, the value range of 16bit encoder is 0~65535), this point is regarded as the zero point of the motor angle.
        
        Data field Description Data
        DATA[0] command byte 0x90
        DATA[1] NULL 0x00
        DATA[2] Encoder position low byte DATA[2] = *(uint8_t *)(&encoder)
        DATA[3] Encoder position high byte DATA[3] = *((uint8_t *)(&encoder)+1)
        DATA[4] Encoder original position low byte DATA[4] = *(uint8_t *)(&encoderRaw)
        DATA[5] Encoder original position high byte DATA[5] = *((uint8_t *)(&encoderRaw)+1)
        DATA[6] Encoder zero offset low byte DATA[6] = *(uint8_t *)(&encoderOffset)
        DATA[7] Encoder zero offset high byte DATA[7] = *((uint8_t *)(&encoderOffset)+1)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_SINGLETURN_ENCODER_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        current_position = (msg.data[2] | (msg.data[3] << 8)) * (360 / 65535)
        original_position = (msg.data[4] | (msg.data[5] << 8)) * (360 / 65535)
        encoder_offset = (msg.data[6] | (msg.data[7] << 8)) * (360 / 65535)
        if debug != None:
            print(f"Motor position: {current_position}")
            print(f"Motor original position: {original_position}")
            print(f"Motor offset: {encoder_offset}")
        return current_position, original_position, encoder_offset
    
    def write_encoder_value_to_ROM_as_motor_zero(self,offset,debug=None):
        """
        The host sends this command to set the encoder zero offset, where the encoder value encoderOffset to be written is uint16_t type, and the value range of the 16bit encoder is 0~65535. 
        
        Data field Description Data
        DATA[0] command byte 0x91
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] Encoder zero offset low byte DATA[6] = *(uint8_t *)(&encoderOffset)
        DATA[7] Encoder zero offset high byte DATA[7] = *((uint8_t *)(&encoderOffset)+1)
        
        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data is the same as the host sent.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_ENCODER_VALUES_TO_ROM_AS_MOTOR_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, (offset) & 0xFF, ((offset) & 0xFF00) >> 8])
        new_encoder_offset = (msg.data[6] | (msg.data[7] << 8))
        if debug != None: print(f"Encoder offset set to {new_encoder_offset}")
        return new_encoder_offset

    def write_singleturn_encoder_current_position_to_ROM_as_motor_zero(self,debug=None):
        """
        Write the current encoder position of the motor as the initial position to the ROM
        
        Notice:
        1.This command needs to be re-powered to take effect
        2.This command will write the zero point to the ROM of the drive. Multiple writes will reduce thelifeof thechip. Frequent use is not recommended.
        
        Data field Description Data
        DATA[0] command byte 0x19
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame) :
        The motor responds to the host after receiving the command, the encoderOffset in the data is thezerooffset value
        
        Data field Description Data
        DATA[0] command byte 0x19
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] Encoder zero offset low byte DATA[6] = *(uint8_t *)(&encoderOffset)
        DATA[7] Encoder zero offset high byte DATA[7] = *((uint8_t *)(&encoderOffset)+1)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_WRITE_SINGLETURN_ENCODER_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_offset = (msg.data[6] | (msg.data[7] << 8))
        if debug != None: print(f"Initial position set to {encoder_offset}")
        return encoder_offset
    
    def read_multiturn_encoder_angle(self,debug=None):
        """
        The host sends command to read the multi-turn angle of the motor. 
        
        Data field Description Data
        DATA[0] command byte 0x92
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters. 
        motor Angle, (int64_t type,value range,valid data is 6 bytes),the seventh byte represent positiveandnegative, 0 is positive and 1 is negative, unit 0.01°/LSB

        Data field Description Data
        DATA[0] command byte 0x92
        DATA[1] Angle low byte 1 DATA[1] = *(uint8_t *)(&motorAngle)
        DATA[2] Angle byte2 DATA[2] = *((uint8_t *)(& motorAngle)+1)
        DATA[3] Angle byte3 DATA[3] = *((uint8_t *)(& motorAngle)+2)
        DATA[4] Angle byte4 DATA[4] = *((uint8_t *)(& motorAngle)+3)
        DATA[5] Angle byte5 DATA[5] = *((uint8_t *)(& motorAngle)+4)
        DATA[6] Angle byte6 DATA[6] = *((uint8_t *)(& motorAngle)+5)
        DATA[7] Motor angle positive or negative flag bit 1 negative, 0 positive
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_MULTITURN_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        motor_angle = msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40) * 0.01
        if msg.data[7] != 0 : motor_angle = -motor_angle
        if debug !=None : print(f"Encoder multiturn angle: {motor_angle:.2f}°",end='\r')
        return motor_angle

    def read_singleturn_encoder_angle(self,debug=None):
        """
        The host sends command to read the single circle angle of the motor. 
        
        Data field Description Data
        DATA[0] command byte 0x94
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters. 
        The motor single circle angle is int16_t type data. 
        It starts from the encoder zero point and increasesclockwise. 
        When the zero point is reached again, the value returns to 0, the unit is 0.01°/LSB, andthevalue range is 0~35999. 
        
        Data field Description Data
        DATA[0] command byte 0x94
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] single angle low byte DATA[6] = *(uint8_t *)(& circleAngle)
        DATA[7] single angle high byte DATA[7] = *((uint8_t *)(& circleAngle)+1)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_SINGLE_CIRCLE_ANGLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        circle_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01
        if debug != None: print(f"Encoder singleturn position: {circle_angle:.2f}°",end='\r')
        return circle_angle


    def read_motor_status_1_error_flag(self,debug=None):
        """
        This command reads the current motor temperature, voltage and error status flag. 
        
        Data field Description Data
        DATA[0] command byte 0x9A
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters. 
        1.Motor temperature (int8_t type, unit 1°C/LSB)
        2.voltage (uint16_t type, unit 0.1V/LSB)
        3.Error State (uint16_t type, each bit represents different motor state)

        Data field Description Data
        DATA[0] command byte 0x9A
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] voltage low byte DATA[3] = *(uint8_t *)(&voltage)
        DATA[5] voltage high byte DATA[4] = *((uint8_t *)(& voltage)+1)
        DATA[6] Error State low byte 1 DATA[6] =*(uint8_t *)(&errorState)
        DATA[7] Error State byte 2 DATA[7] = *((uint8_t *)(& errorState)+1)

        Memo:
        System_errorState value state table 1 is as follows:

        System_errorState value Status description
        0x0000 Hardware over-current
        0x0002 Motor stalled
        0x0004 Low voltage
        0x0008 Over-voltage
        0x0010 Over-current
        0x0020 brake opening failed
        0x0040 Bus current error
        0x0080 Battery voltage error
        0x0100 overspeed
        0x0200 Position loop exceeded error
        0x0400 VDD error
        0x0800 DSP internal sensor temperature is overheated0x1000 motor temperature is overheated
        0x2000 Encoder calibration error

        CAN_errorState value state table 2 is as follows:

        CAN_errorState value Status description
        0x00F0 PID parameter write ROM protection, non-safe operation
        0x00F1 Encoder value is written into ROM protection, non-safe operation0x00F2 Three-loop switching operation error, non-safe operation
        0x00F3 Motor brake is not open
        0x00F4 Motor write ROM protection, non-safe operation
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        motor_temp = msg.data[1]
        voltage = (msg.data[3] | (msg.data[4] << 8)) * 0.1
        error_state = msg.data[6] | (msg.data[7] << 8)
        if debug != None:
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

    def read_motor_status_2(self,debug=None):
        """
        This command reads the current temperature, voltage, speed and encoder position of the motor. 
        
        Data field Description Data
        DATA[0] command byte 0x9C
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters. 
        1.Motor temperature (int8_t type, 1℃/LSB)
        2.Motor torque current Iq (int16_t type, Range:-2048~2048,real torque current range:-33A~33A)
        3.Motor speed (int16_t type, 1dps/LSB)
        4.Encoder position value (uint16_t type, 16bit encoder value range:0~65535)
        
        Data field Description Data
        DATA[0] Command byte 0x9C
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MOTOR_STATUS_2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current: {torque_current}A")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Encoder position: {encoder_position}")
        return motor_temp, torque_current, motor_speed, encoder_position

    def read_motor_status_3(self,debug=None):
        """
        This command reads the current temperature and phase current data of the motor. 
        
        Data field Description Data
        DATA[0] Command byte 0x9D
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters:
        1.Motor temperature (int8_t type, 1℃/LSB)
        2.A phase current data,the data type is int16_t type, corresponding to the actual phase current is1A/64LSB. 
        3.B phase current data,the data type is int16_t type,corresponding to the actual phase current is1A/64LSB.
        4.C phase current data,the data type is int16_t type,corresponding to the actual phase current is1A/64LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0x9D
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Phase A current low byte DATA[2] = *(uint8_t *)(&iA)
        DATA[3] Phase A current high byte DATA[3] = *((uint8_t *)(& iA)+1)
        DATA[4] Phase B current low byte DATA[4] = *(uint8_t *)(&iB)
        DATA[5] Phase B current high byte DATA[5] = *((uint8_t *)(& iB)+1)
        DATA[6] Phase C current low byte DATA[6] = *(uint8_t *)(&iC)
        DATA[7] Phase C current high byte DATA[7] = *((uint8_t *)(& iC)+1)
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, 
                                       data=[self.ADDR_READ_MOTOR_STATUS_3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        motor_temp = msg.data[1]
        phase_A_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        phase_B_current = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        phase_C_current = (msg.data[6] | (msg.data[7] << 8)) * 0.01
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Phase A current: {phase_A_current}A")
            print(f"Phase B current: {phase_B_current}A")
            print(f"Phase C current: {phase_C_current}A")
        return motor_temp, phase_A_current, phase_B_current, phase_C_current


    def motor_off(self,debug=None): 
        """
        Turn off the motor, and clear the running state of the motor and the previously received control
        commands at the same time. 
        
        Data field Description Data
        DATA[0] Command byte 0x80
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent bythe host.
        """
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + self.id, [self.ADDR_MOTOR_OFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if msg.can_id == self.BASE_ADDR_ID + self.id and msg.data[0]==self.ADDR_MOTOR_OFF and msg.data[1]==0x00 and msg.data[2]==0x00 and msg.data[3]==0x00 and msg.data[4]==0x00 and msg.data[5]==0x00 and msg.data[6]==0x00 and msg.data[7]==0x00 : return 1
        else : return 0

    def stop(self,debug=None):
        """
        Stop the motor, but do not clear the running state of the motor and the previously received control
        commands. 
        
        Data field Description Data
        DATA[0] Command byte 0x81
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent bythe host.
        """
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + self.id, [self.ADDR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if msg.can_id == self.BASE_ADDR_ID + self.id and msg.data[0]==self.ADDR_STOP and msg.data[1]==0x00 and msg.data[2]==0x00 and msg.data[3]==0x00 and msg.data[4]==0x00 and msg.data[5]==0x00 and msg.data[6]==0x00 and msg.data[7]==0x00 : return 1
        else : return 0

    def running(self,debug=None):
        """
        Resume motor operation from the motor stop command (recover the control mode before thestop). 
        
        Data field Description Data
        DATA[0] Command byte 0x88
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive reply (one frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent by the host.
        """
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_RUNNING, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        if msg.can_id == self.BASE_ADDR_ID + self.id and msg.data[0]==self.ADDR_RUNNING and msg.data[1]==0x00 and msg.data[2]==0x00 and msg.data[3]==0x00 and msg.data[4]==0x00 and msg.data[5]==0x00 and msg.data[6]==0x00 and msg.data[7]==0x00 : return 1
        else : return 0


    def torque_closed_loop_control(self, torque,debug=None):
        """
        The host sends this command to control the torque current output of the motor. 
        The control valueiqControl is type of int16_t, and the value range is -2000~2000, corresponding to the actual torquecurrent range -32A~32A (the bus current and the actual torque of the motor vary with different motors. ). 
        
        Data field Description Data
        DATA[0] Command byte 0xA1
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Low byte of torque current control value DATA[4] = *(uint8_t *)(&iqControl)
        DATA[5] high byte of torque current control value DATA[5] = *((uint8_t *)(&iqControl)+1)
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Memo:
        The control value iqControl in this command is not limited by the Max Torque Current value in the host
        computer of debugging software. 
        
        Drive reply (one frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters：
        1.Motor temperature(int8_t type, 1℃/LSB). 
        2.Motor torque current Iq (int16_t type, Range-2048~2048, corresponding to the actual torque current range -33A~33A). 
        3.Motor speed (int16_t type,1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA1
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)

        Note: During the three-loop switching process, if the motor is not in the safe state, the drive will returnthethree-loop operation error value, 0x00F6, and the motor will switch to the current-loop safe state. 
        Pleasebe aware that the following three-loop operation instructions are similar. 
        
        Non-safe operation error response:

        Data field Description Data
        DATA[0] Command byte 0xA1
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] Error status low byte DATA[6] =*(uint8_t *)(&errorState)
        DATA[7] Error status high byte DATA[6] =*((uint8_t *)(&errorState)+1)
        """
        torque = int(torque * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_TORQUE_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00, (torque >> 0) & 0xFF, (torque >> 8) & 0xFF, 0x00, 0x00])
        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current value (-2048~2048): {torque_current}")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor position value (0~16383): {encoder_position}")
        return motor_temp, torque_current, motor_speed, encoder_position
    
    def speed_closed_loop_control(self, speed,debug=None):
        """
        The host sends this command to control the speed of the motor. 
        The control value speedControl isoftype int32_t, corresponding to the actual speed of 0.01dps/LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0xA2
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Speed control low byte DATA[4] = *(uint8_t *)(&speedControl)
        DATA[5] Speed control DATA[5] = *((uint8_t *)(&speedControl)+1)
        DATA[6] Speed control DATA[6] = *((uint8_t *)(&speedControl)+2)
        DATA[7] Speed control high byte DATA[7] = *((uint8_t *)(&speedControl)+3)

        Memo:
        1.The maximum torque current of the motor under this command is limited by the Max TorqueCurrent value in the host computer of debugging software. 
        2.In this control mode, the maximum acceleration of the motor is limited by the Max Accelerationvalueinthe host computer of debugging software. 
        
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters :
        1.Motor temperature(int8_t type, 1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3.Motor speed'int16_t type,1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA2
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        speed = int(speed * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_SPEED_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00, (speed >> 0) & 0xFF, (speed >> 8) & 0xFF, (speed >> 16) & 0xFF, (speed >> 24) & 0xFF])
        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current value (-2048~2048): {torque_current}")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor position value (0~16383): {encoder_position}")
        return motor_temp, torque_current, motor_speed, encoder_position

    def position_closed_loop_control_1(self, position,debug=None):
        """
        The host sends this command to control the position of the motor (multi-turn angle), the control valueangleControl is type of int32_t, and the corresponding actual position is 0.01degree/LSB, that is, 36000represents 360°, and the direction of rotation of the motor is determined by the difference betweenthetarget position and the current position. 
        
        Data field Description Data
        DATA[0] Command byte 0xA3
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] Position control DATA[6] = *((uint8_t *)(&angleControl)+2)
        DATA[7] Position control high byte DATA[7] = *((uint8_t *)(&angleControl)+3)

        Memo :
        1.The control value angleControl under this command is limited by the Max Angle value in the host computer of debugging software. 
        2.The maximum speed of the motor under this command is limited by the Max Speed value in the upper computer of debugging software. 
        3.In this control mode, the maximum acceleration of the motor is limited by the Max Accelerationvalueinthe host computer of debugging software. 
        4.In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer of debugging software. 
        
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters :
        1.Motor temperature (int8_t type,1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3.Motor speed (int16_t type, 1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA3
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        position_control = int(position * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_POSITION_MULTITURN_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00,(position_control >> 0) & 0xFF, (position_control >> 8) & 0xFF,(position_control >> 16) & 0xFF, (position_control >> 24) & 0xFF])
        while True:
            motor_angle = self.read_multiturn_encoder_angle(debug=True)
            motor_temp, torque_current, motor_speed, encoder_position = self.read_motor_status_2()
            if motor_angle>position-0.1 and motor_angle<position+0.1 and motor_speed==0.0: break
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current: {torque_current}A")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor angle: {motor_angle}°")
        return motor_temp, torque_current, motor_speed, motor_angle
	
    def position_closed_loop_control_2(self, speed, position,debug=None):
        """
        The host sends this command to control the position of the motor (multi-turn angle), the control valueangleControl is of type int32_t, and the corresponding actual position is 0.01degree/LSB, that is, 36000represents 360°, and the direction of rotation of the motor is determined by the difference betweenthetarget position and the current position. 
        The control value maxSpeed limits the maximum speed of motor rotation, which is of typeuint16_t,corresponding to the actual speed of 1dps/LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0xA4
        DATA[1] NULL 0x00
        DATA[2] Speed limit low byte DATA[2] = *(uint8_t *)(&maxSpeed)
        DATA[3] Speed limit high byte DATA[3] = *((uint8_t *)(&maxSpeed)+1)
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] Position control DATA[6] = *((uint8_t *)(&angleControl)+2)
        DATA[7] Position control high byte DATA[7] = *((uint8_t *)(&angleControl)+3)

        Memo :

        1.The control value angleControl under this command is limited by the Max Angle value in the host computer. 
        2.In this control mode, the maximum acceleration of the motor is limited by the Max Accelerationvalueinthe host computer. 
        3.In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer. 
        
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters:
        1.Motor temperature (int8_t type, 1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3.Motor speed (int16_t type, 1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA4
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        max_speed = speed & 0xFFFF
        angle_control = int(position * 100)
        msg=self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL, 0x00, (max_speed >> 0) & 0xFF, (max_speed >> 8) & 0xFF, (angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF, (angle_control >> 16) & 0xFF, (angle_control >> 24) & 0xFF])
        while True:
            motor_angle = self.read_multiturn_encoder_angle(debug=True)
            motor_temp, torque_current, motor_speed, encoder_position = self.read_motor_status_2()
            if motor_angle>position-0.1 and motor_angle<position+0.1 and motor_speed==0.0: break
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current: {torque_current}A")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor angle: {motor_angle}°")
        return motor_temp, torque_current, motor_speed, motor_angle
	
    def position_closed_loop_control_3(self, direction, position,debug=None):
        """
        The host sends this command to control the position of the motor (single-turn angle), the control valueangleControl is of uint16_t type, the value range is 0~35999, and the corresponding actual positionis0.01degree/LSB, that is, the actual angle range is 0°~359.99°. 
        The control value spinDirection sets the direction of motor rotation, which is of type uint8_t, 0x00meansclockwise, 0x01 means counterclockwise. 
        
        Data field Description Data
        DATA[0] Command byte 0xA5
        DATA[1] Rotation direction byte DATA[1] = spinDirection
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control high byte DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Memo:
        1.The maximum speed of the motor under this command is limited by the Max Speed value in the host computer. 
        2.In this control mode, the maximum acceleration of the motor is limited by the Max Accelerationvalueinthe host computer. 
        3.In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer.

        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters :
        1.Motor temperature (int8_t type, 1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3.Motor speed (int16_t type,1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA5
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        angle_control = int(position * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_POSITION_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL,direction & 0xFF, 0x00, 0x00,(angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF, 0x00, 0x00])
        while True:
            motor_angle = self.read_singleturn_encoder_angle(debug=True)
            motor_temp, torque_current, motor_speed, encoder_position = self.read_motor_status_2()
            if motor_angle>position-0.1 and motor_angle<position+0.1 and motor_speed==0.0: break
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current: {torque_current}A")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor angle: {motor_angle}°")
        return motor_temp, torque_current, motor_speed, motor_angle

    def position_closed_loop_control_4(self, speed, direction, position,debug=None):
        """
        The host sends this command to control the position of the motor (single turn angle). 
        1.AngleControl is of uint16_t type, the value range is 0~35999, and the corresponding actual positionis0.01degree/LSB, that is, the actual angle range is 0°~359.99°. 
        2.spinDirection sets the direction of motor rotation, which is of uint8_t type, 0x00 means clockwise, 0x01means counterclockwise
        3.maxSpeed limits the maximum speed of motor rotation, which is of uint16_t type, correspondingtotheactual speed of 1dps/LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0xA6
        DATA[1] Rotation direction byte DATA[1] = spinDirection
        DATA[2] Speed limit low byte DATA[2] = *(uint8_t *)(&maxSpeed)
        DATA[3] Speed limit high byte DATA[3] = *((uint8_t *)(&maxSpeed)+1)
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control high byte DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Memo:
        1.In this control mode, the maximum acceleration of the motor is limited by the Max Accelerationvalueinthe host computer. 
        2.In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer.
        
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters:
        1. Motor temperature (int8_t type, 1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3.Motor speed (int16_t type, 1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535)

        Data field Description Data
        DATA[0] Command byte 0xA6
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """
        angle_control = int(position * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_POSITION_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL,direction & 0xFF,(speed >> 0) & 0xFF, (speed >> 8) & 0xFF,
(angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF,0x00, 0x00])
        while True:
            motor_angle = self.read_singleturn_encoder_angle(debug=True)
            motor_temp, torque_current, motor_speed, encoder_position = self.read_motor_status_2()
            if motor_angle>position-0.1 and motor_angle<position+0.1 and motor_speed==0.0: break
        if debug!=None :
            print(f"Motor temperature: {motor_temp}°C")
            print(f"Torque current: {torque_current}A")
            print(f"Motor speed: {motor_speed}dps")
            print(f"Motor angle: {motor_angle}°")
        return motor_temp, torque_current, motor_speed, motor_angle
    
    def position_closed_loop_control_5(self,angleControl,debug=None):
        """
        The host sends this command to control the incremental position of the motor (multi-turn angle), andrunthe input position increment with the current position as the starting point. 
        The control value angleControl is of type int32_t, and the corresponding actual position is 0.01degree/LSB, that is, 36000represents360° , 
        The direction of motor rotation is determined by the incremental position sign. 
        
        Data field Description Data
        DATA[0] Command byte 0xA7
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] Position control DATA[6] = *((uint8_t *)(&angleControl)+2)
        DATA[7] Position control high byte DATA[7] = *((uint8_t *)(&angleControl)+3)

        Memo:
        1. The control value angleControl under this command is limited by the Max Angle value in the host computer. 
        2. The maximum speed of the motor under this command is limited by the Max Speed value in the host computer. 
        3. In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host computer. 
        4. In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer. 
        
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the following parameters:
        1. motor temperature (int8_t type, 1℃/LSB)
        2. Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A). 
        3. motor speed(int16_t type, 1dps/LSB). 4. Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535)
        
        Data field Description Data
        DATA[0] Command byte 0xA7
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """

    def position_closed_loop_control_6(self,maxSpeed,angleControl,debug=None):
        """
        The host sends this command to control the incremental position (multi-turn angle) of themotor, andruns the input position increment with the current position as the starting point. 
        The control valueangleControl is of type int32_t, and the corresponding actual position is 0.01degree/LSB, that is, 36000represents 360°, and the motor rotation direction is determined by the incremental position sign. 
        The control value maxSpeed limits the maximum speed of motor rotation, which is of typeuint16_t,corresponding to the actual speed of 1dps/LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0xA8
        DATA[1] NULL 0x00
        DATA[2] Speed limit low byte DATA[2] = *(uint8_t *)(&maxSpeed)
        DATA[3] Speed limit high byte DATA[3] = *((uint8_t *)(&maxSpeed)+1)
        DATA[4] Position control low byte DATA[4] = *(uint8_t *)(&angleControl)
        DATA[5] Position control DATA[5] = *((uint8_t *)(&angleControl)+1)
        DATA[6] Position control DATA[6] = *((uint8_t *)(&angleControl)+2)
        DATA[7] Position control high byte DATA[7] = *((uint8_t *)(&angleControl)+3)

        Memo:
        1.The control value angleControl under this command is limited by the Max Angle value in the host computer. 
        2. In this control mode, the maximum acceleration of the motor is limited by the Max Acceleration value in the host computer. 
        3. In this control mode, the maximum torque current of the motor is limited by the Max TorqueCurrent value in the host computer.
         
        Drive response (1 frame)
        The motor responds to the host after receiving the command, the frame data contains the followingparameters:
        1.Motor temperature (int8_t type, 1℃/LSB)
        2.Motor torque current(Iq)(int16_t type, range -2048~2048, corresponding to actual torque current range-33A~33A)
        3.Motor speed (int16_t type, 1dps/LSB)
        4.Encoder position value (uint16_t type, the value range of 16bit encoder is 0~65535). 
        
        Data field Description Data
        DATA[0] Command byte 0xA8
        DATA[1] Motor temperature DATA[1] = *(uint8_t *)(&temperature)
        DATA[2] Torque current low byte DATA[2] = *(uint8_t *)(&iq)
        DATA[3] Torque current high byte DATA[3] = *((uint8_t *)(&iq)+1)
        DATA[4] Motor speed low byte DATA[4] = *(uint8_t *)(&speed)
        DATA[5] Motor speed high byte DATA[5] = *((uint8_t *)(&speed)+1)
        DATA[6] Encoder position low byte DATA[6] = *(uint8_t *)(&encoder)
        DATA[7] Encoder position high byte DATA[7] = *((uint8_t *)(&encoder)+1)
        """


    def system_operation_mode_aquisition(self,debug=None):
        """
        This command reads the current motor running mode. Data field Description Data
        DATA[0] Command byte 0x70
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor responds to the host after receiving the command, and the drive reply data contains theparameter run mode operating status, which is of type uint8_t. 
        The motor operation mode has the following 4 states :
        1.Current loop mode(0x00). 
        2.Speed loop mode(0x01). 
        3.Position loop mode(0x02). 
        4.Power-on initialization state, not in three-ring mode(0xFF).

        Data field Description Data
        DATA[0] Command byte 0x70
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] Motor running mode DATA[7] = *(uint8_t *)(&runmode)
        """

    def motor_power_acquisition(self,debug=None):
        """
        This command reads the current motor running mode. 
        
        Data field Description Data
        DATA[0] Command byte 0x71
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor responds to the host after receiving the command. 
        The drive response data containsthemotor power parameter motor power, which is of type uint16_t, the unit is watts, and the unit is 0.1w/LSB. 
        
        Data field Description Data
        DATA[0] Command byte 0x71
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] Motor running power low byte DATA[6] = *(uint8_t *)(&motorpower)
        DATA[7] Motor running power high byte DATA[7] = *((uint8_t *)(&motorpower)+1)
        """

    def get_battery_voltage_value(self,debug=None):
        """
        This command reads the current auxiliary battery voltage. 
        
        Data field Description Data
        DATA[0] Command byte 0x72
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor will reply to the host after receiving the command. 
        The driver reply data contains theauxiliarybattery voltage parameter batvoltage, which is of type uint8_t, the unit is volts, and the unit is 0.1v/LSB

        Data field Description Data
        DATA[0] Command byte 0x72
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] High battery voltage DATA[7] = *(uint8_t *)(&batvoltage)
        """

    def feedforward_setting(self,TFCurrent,encoder,debug=None):
        """
        This command sets the current feedforward current size, the parameters are as follows. 
        1.TF feedforward current value TFCurrent of the motor (int16_t type,unit A, zoom in 100 times in current mode, such as 10 corresponds to 0.1A)
        2.Encoder position multi-turn encoder (int32_t type). 
        
        Data field Description Data
        DATA[0] Command byte 0x73
        DATA[1] NULL 0x00
        DATA[2] TF feedforward current value low byte DATA[6] = *(uint8_t *)(&TFCurrent)
        DATA[3] TF feedforward current value high byte DATA[7] = *((uint8_t *)(&TFCurrent)+1)
        DATA[4] Encoder position low byte 1 DATA[4] = *(uint8_t *)(&encoder)
        DATA[5] Encoder position low byte 2 DATA[5] = *((uint8_t *)(&encoder)+1)
        DATA[6] Encoder position low byte 3 DATA[6] = *((uint8_t *)(&encoder)+2)
        DATA[7] Encoder position low byte 4 DATA[7] = *((uint8_t *)(&encoder)+3)

        Drive response (1 frame)
        The motor responds to the host after receiving the command,the following parameters are includedinthe driver response data. 
        1. Motor torque current(Iq)(int16_t type, zoom in 100 times in current mode, such as 10 correspondsto0.1A)
        2. motor speed (int16_t type, 1dps/LSB)
        3. Encoder position multi-turn encoder (int32_t type). 
        
        Data field Description Data
        DATA[0] Torque current low byte DATA[0] = *(uint8_t *)(&iq)
        DATA[1] Torque current high byte DATA[1] = *((uint8_t *)(&iq)+1)
        DATA[2] Motor speed low byte DATA[2] = *(uint8_t *)(&speed)
        DATA[3] Motor speed low byte DATA[3] = *((uint8_t *)(&speed)+1)
        DATA[4] Encoder position low byte 1 DATA[4] = *(uint8_t *)(&encoder)
        DATA[5] Encoder position byte 2 DATA[5] = *((uint8_t *)(&encoder)+1)
        DATA[6] Encoder position byte 3 DATA[6] = *((uint8_t *)(&encoder)+2)
        DATA[7] Encoder position byte 4 DATA[7] = *((uint8_t *)(&encoder)+3)
        """

    def system_reset(self,debug=None):
        """
        This command is used to reset the system software. 
        
        Data field Description Data
        DATA[0] Command byte 0x76
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent by the host. 
        
        Data field Description Data
        DATA[0] Command byte 0x76
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00
        """

    def system_brake_opening(self,debug=None):
        """
        This command is used to open the system brake. 
        
        Data field Description Data
        DATA[0] Command byte 0x77
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent bythe host. 
        
        Data field Description Data
        DATA[0] Command byte 0x77
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00
        """

    def system_brake_close(self,debug=None):
        """
        This command is used to open the system brake. 
        
        Data field Description Data
        DATA[0] Command byte 0x78
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00

        Drive response (1 frame)
        The motor responds to the host after receiving the command,the frame data is the same as that sent bythe host. 
        
        Data field Description Data
        DATA[0] Command byte 0x78
        DATA[1] NULL 0x00
        DATA[2] NULL 0x00
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] NULL 0x00
        """

    def CAN_ID(self,debug=None):
        """
        This command is used to set and read CAN ID. The host sends this command to set and read CAN ID. 
        The parameters are as follows. 
        1.The read and write flag bit is bool type, 1 read 0 write. 
        2.CANID,Size range (#1~#32), uint16_t type (synchronized with the host computer function), deviceidentifier 0x140 + ID (1~32). 
        
        Data field Description Data
        DATA[0] Command byte 0x79
        DATA[1] NULL 0x00
        DATA[2] Read and write flags DATA[2] = wReadWriteFlag
        DATA[3] NULL 0x00
        DATA[4] CANID low byte1 DATA[4] = *(uint8_t *)(&CANID)
        DATA[5] CANID byte 2 DATA[5] = *((uint8_t *)(&CANID)+1)
        DATA[6] CANID byte 3 DATA[6] = *((uint8_t *)(&CANID)+2)
        DATA[7] CANID byte 4 DATA[7] = *((uint8_t *)(&CANID)+3)
        
        Driver reply (one frame)
        1.The motor responds to the host after receiving the command,which is divided into the followingtwoSituations. 
        2.Set CANID, range 1-32, and return to the original command. 
        3.Read CANID, return parameters are as follows. 
        
        Data field Description Data
        DATA[0] Command byte 0x79
        DATA[0] NULL 0x00
        DATA[0] Read and write flags DATA[2] = wReadWriteFlag
        DATA[0] NULL 0x00
        DATA[4] CANID low byte 1 DATA[4] = *(uint8_t *)(&CANID)
        DATA[5] CANID byte 2 DATA[5] = *((uint8_t *)(&CANID)+1)
        DATA[6] CANID byte 3 DATA[6] = *((uint8_t *)(&CANID)+2)
        DATA[7] CANID byte 4 DATA[7] = *((uint8_t *)(&CANID)+3)
        """
    
    def multiple_motor_torque_closed_loop(self,debug=None):
        """"
        The format of the message used to send commands to multiple motors at the same time, as followed:
        Identifier : 0x280
        Frame format : DATA
        Frame type : standard frame
        DLC : 8byte
        The host simultaneously send this command to control the torque current output up to 4 motors. 
        Thecontrol value iqControl is int16_t type, the value range is -2000~2000, corresponding to the actual torquecurrent range -32A~32A 
        (The bus current and the actual torque of the motor vary from motor tomotor). 
        The motor ID should be set to #1~#4, and cannot be repeated, corresponding to the 4 torquecurrentsinthe frame data. 
        
        Data field Description Data
        DATA[0] Torque current 1 control value low byte DATA[0] = *(uint8_t *)(&iqControl_1)
        DATA[1] Torque current 1 control value high byte DATA[1] = *((uint8_t *)(&iqControl_1)+1)
        DATA[2] Torque current 2 control value low byte DATA[2] = *(uint8_t *)(&iqControl_2)
        DATA[3] Torque current 2 control value high byte DATA[3] = *((uint8_t *)(&iqControl_2)+1)
        DATA[4] Torque current 3 control value low byte DATA[4] = *(uint8_t *)(&iqControl_3)
        DATA[5] Torque current 3 control value high byte DATA[5] = *((uint8_t *)(&iqControl_3)+1)
        DATA[6] Torque current 4 control value low byte DATA[6] = *(uint8_t *)(&iqControl_4)
        DATA[7] Torque current 4 control value high byte DATA[7] = *((uint8_t *)(&iqControl_4)+1)

        Driver reply (one frame)
        The message format of each motor reply command is as follows:
        Identifier : 0x140 + ID(1~4)
        Frame format : DATA
        Frame type : standard frame
        DLC : 8byte
        Each motor reply according to the ID from small to large, and the reply data of each motor isthesame as the single motor torque closed-loop control command reply data.
        """