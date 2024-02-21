from Real import CanBusGsUsb

class ProtocolLSeries(CanBusGsUsb):
    # Attributes
    BASE_ADDR_ID = 0x140

    ADDR_READ_PID_PARAMETER_TO_RAM = 0x30
    ADDR_WRITE_PID_PARAMETER_TO_RAM = 0x31
    ADDR_WRITE_PID_PARAMETER_TO_ROM = 0x32
    ADDR_READ_ACCELERATION = 0x33
    ADDR_WRITE_ACCELERATION_TO_RAM = 0x34

    ADDR_SHUTDOWN = 0x80
    ADDR_STOP = 0x81
    ADDR_RUNNING = 0x88

    ADDR_READ_ENCODER_DATA = 0x90
    ADDR_WRITE_ENCODER_OFFSET = 0x91
    ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION = 0x19
    ADDR_READ_MULTITURN_ENCODER_POSITION = 0x92
    ADDR_READ_SINGLETURN_ENCODER_POSITION = 0x94

    ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG = 0x9A
    ADDR_READ_CLEAR_MOTOR_ERROR_FLAG = 0x9B
    ADDR_READ_MOTOR_STATUS_2 = 0x9C
    ADDR_READ_MOTOR_STATUS_3 = 0x9D

    ADDR_TORQUE_CLOSED_LOOP_CONTROL = 0xA1
    ADDR_SPEED_CLOSED_LOOP_CONTROL = 0xA2
    ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL = 0xA3
    ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL = 0xA4
    ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL = 0xA5
    ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL = 0xA6

    MOTOR_STALL = 0x0002
    LOW_PRESSURE = 0x0004
    OVERVOLTAGE = 0x0008
    OVERCURRENT = 0x0010
    POWER_OVERRUN = 0x0040
    SPEEDING = 0x0100
    MOTOR_TEMP_OVER = 0x1000
    ENCODER_CALIB_ERROR = 0x2000

    def __init__(self,id:int,reducer_ratio:int,can_bus:CanBusGsUsb):
        self.id=id
        self.reducer_ratio=reducer_ratio
        self.can_bus = can_bus



    def read_pid_parameter_to_RAM(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_PID_PARAMETER_TO_RAM, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print(msg)

        current_loop_KP = msg.data[2]
        current_loop_KI = msg.data[3]
        speed_loop_KP = msg.data[4]
        speed_loop_KI = msg.data[5]
        position_loop_KP = msg.data[6]
        position_loop_KI = msg.data[7]

        print(f"Current loop KP parameters: {current_loop_KP}")
        print(f"Current loop KI parameters: {current_loop_KI}")
        print(f"Speed loop KP parameters: {speed_loop_KP}")
        print(f"Speed loop KI parameters: {speed_loop_KI}")
        print(f"Position loop KP parameters: {position_loop_KP}")
        print(f"Position loop KI parameters: {position_loop_KI}")

        return current_loop_KP, current_loop_KI, speed_loop_KP, speed_loop_KI, position_loop_KP, position_loop_KI

    def write_pid_parameter_to_RAM(self, current_loop_kp, current_loop_ki, speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_PID_PARAMETER_TO_RAM, 0x00, current_loop_kp, current_loop_ki, speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki])
        print("Current loop KP set to ", msg.data[2])
        print("Current loop KI set to ", msg.data[3])
        print("Speed loop KP set to ", msg.data[4])
        print("Speed loop KI set to ", msg.data[5])
        print("Position loop KP set to ", msg.data[6])
        print("Position loop KI set to ", msg.data[7])
        return msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]

    def write_pid_parameter_to_ROM(self, current_loop_kp, current_loop_ki, speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_PID_PARAMETER_TO_ROM, 0x00, current_loop_kp, current_loop_ki, speed_loop_kp, speed_loop_ki, position_loop_kp, position_loop_ki])
        print("Current loop KP set to ", msg.data[2])
        print("Current loop KI set to ", msg.data[3])
        print("Speed loop KP set to ", msg.data[4])
        print("Speed loop KI set to ", msg.data[5])
        print("Position loop KP set to ", msg.data[6])
        print("Position loop KI set to ", msg.data[7])
        return msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]

    def read_acceleration(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_ACCELERATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print(msg)
        acceleration_data = (msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24))
        print(f"Acceleration: {acceleration_data} dps/s")
        return acceleration_data

    def write_acceleration_to_RAM(self, acceleration):
        accel_bytes = [(acceleration >> 0) & 0xFF,(acceleration >> 8) & 0xFF,(acceleration >> 16) & 0xFF,(acceleration >> 24) & 0xFF]
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_ACCELERATION_TO_RAM, 0x00, 0x00, 0x00, accel_bytes[0], accel_bytes[1], accel_bytes[2], accel_bytes[3]])
        print("RX : Acceleration set to ", msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24))
        return msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
    


    def shutdown(self): 
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + self.id, [self.ADDR_SHUTDOWN, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("RX : Motor shutdown successful" if msg else "RX : Error during motor shutdown")

    def stop(self):
        msg = self.can_bus.sendMessage(self.BASE_ADDR_ID + self.id, [self.ADDR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("RX : Motor stop successful" if msg else "RX : Error during motor stop")

    def running(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID, data=[self.ADDR_RUNNING, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print("Received frame data:", msg.data)



    def read_encoder_data(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_ENCODER_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        current_position = (msg.data[2] | (msg.data[3] << 8))
        original_position = (msg.data[4] | (msg.data[5] << 8))
        encoder_offset = (msg.data[6] | (msg.data[7] << 8))

        print(f"Motor position: {current_position}")
        print(f"Motor original position: {original_position}")
        print(f"Motor offset: {encoder_offset}")

        return current_position, original_position, encoder_offset

    def write_encoder_offset(self, offset):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_ENCODER_OFFSET, 0x00, 0x00, 0x00, 0x00, 0x00, (offset) & 0xFF, ((offset) & 0xFF00) >> 8])
        new_encoder_offset = (msg.data[6] | (msg.data[7] << 8))
        print(f"RX: encoder offset set to {new_encoder_offset}")
        return new_encoder_offset

    def write_current_position_to_ROM_as_motor_zero_position(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        encoder_offset = (msg.data[6] | (msg.data[7] << 8))
        print(f"RX: initial position set to {encoder_offset}")
        return encoder_offset

    def read_multiturn_encoder_position(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_MULTITURN_ENCODER_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        motor_angle = (msg.data[1] | (msg.data[2] << 8) | (msg.data[3] << 16) | (msg.data[4] << 24) | (msg.data[5] << 32) | (msg.data[6] << 40) | (msg.data[7] << 48)) * 0.01
        print(f"Encoder multiturn position: {motor_angle}°")
        return motor_angle

    def read_singleturn_encoder_position(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_SINGLETURN_ENCODER_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        circle_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01
        print(f"Encoder singleturn position: {circle_angle}°")
        return circle_angle


    
    def read_motor_status_1_error_flag(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
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

    def read_clear_motor_error_flag(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_CLEAR_MOTOR_ERROR_FLAG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
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
            if error_state & self.LOW_PRESSURE:
                print("Voltage status: low voltage protection")
            if error_state & self.OVERVOLTAGE:
                print("Voltage status: overvoltage protection")
            if error_state & self.OVERCURRENT:
                print("Voltage status: overcurrent protection")
            if error_state & self.POWER_OVERRUN:
                print("Voltage status: power overrun")
            if error_state & self.SPEEDING:
                print("Voltage status: speeding")
            if error_state & self.MOTOR_TEMP_OVER:
                print("Temperature status: over temperature protection")
            if error_state & self.ENCODER_CALIB_ERROR:
                print("Error status: encoder calibration error")
        
        return motor_temp, voltage, error_state

    def read_motor_status_2(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_MOTOR_STATUS_2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print(msg)

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))

        print(f"Motor temperature: {motor_temp}°C")
        print(f"Torque current: {torque_current}A")
        print(f"Motor speed: {motor_speed}dps")
        print(f"Encoder position: {encoder_position}")

        return motor_temp, torque_current, motor_speed, encoder_position
    
    def read_motor_status_3(self):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_READ_MOTOR_STATUS_3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        print(msg)

        motor_temp = msg.data[1]
        phase_A_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        phase_B_current = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        phase_C_current = (msg.data[6] | (msg.data[7] << 8)) * 0.01

        print(f"Motor temperature: {motor_temp}°C")
        print(f"Phase A current: {phase_A_current}A")
        print(f"Phase B current: {phase_B_current}A")
        print(f"Phase C current: {phase_C_current}A")

        return motor_temp, phase_A_current, phase_B_current, phase_C_current


    
    def torque_closed_loop_control(self, torque):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_TORQUE_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00, (torque >> 0) & 0xFF, (torque >> 8) & 0xFF, 0x00, 0x00])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current value (-2048~2048): {torque_current}")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor position value (0~16383): {encoder_position}")

        return motor_temp, torque_current, motor_speed, encoder_position
    
    def speed_closed_loop_control(self, speed):
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_SPEED_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00, (speed >> 0) & 0xFF, (speed >> 8) & 0xFF, (speed >> 16) & 0xFF, (speed >> 24) & 0xFF])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8))
        motor_speed = (msg.data[4] | (msg.data[5] << 8))
        encoder_position = (msg.data[6] | (msg.data[7] << 8))

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current value (-2048~2048): {torque_current}")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor position value (0~16383): {encoder_position}")

        return motor_temp, torque_current, motor_speed, encoder_position

    def write_position_multiturn_closed_loop_control(self, position):
        position_control = int(position * 100)
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL, 0x00, 0x00, 0x00,
                                                                    (position_control >> 0) & 0xFF, (position_control >> 8) & 0xFF,
                                                                    (position_control >> 16) & 0xFF, (position_control >> 24) & 0xFF])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        motor_speed = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        motor_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current: {torque_current}A")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor angle: {motor_angle}°")

        return motor_temp, torque_current, motor_speed, motor_angle

    def write_position_multiturn_speed_closed_loop_control(self, speed, position):
        max_speed = speed & 0xFFFF
        angle_control = int(position * 100)

        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL, 0x00,
                                                                    (max_speed >> 0) & 0xFF, (max_speed >> 8) & 0xFF,
                                                                    (angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF,
                                                                    (angle_control >> 16) & 0xFF, (angle_control >> 24) & 0xFF])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        motor_speed = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        motor_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current: {torque_current}A")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor angle: {motor_angle}°")

        return motor_temp, torque_current, motor_speed, motor_angle

    def write_position_singleturn_direction_closed_loop_control(self, direction, position):
        angle_control = int(position * 100)
        
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL,
                                                                    direction & 0xFF, 0x00, 0x00,
                                                                    (angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF,
                                                                    0x00, 0x00])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        motor_speed = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        motor_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current: {torque_current}A")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor angle: {motor_angle}°")

        return motor_temp, torque_current, motor_speed, motor_angle

    def write_position_singleturn_speed_direction_closed_loop_control(self, speed, direction, position):
        angle_control = int(position * 100)
        
        msg = self.can_bus.sendMessage(id=self.BASE_ADDR_ID + self.id, data=[self.ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL,
                                                                    direction & 0xFF,
                                                                    (speed >> 0) & 0xFF, (speed >> 8) & 0xFF,
                                                                    (angle_control >> 0) & 0xFF, (angle_control >> 8) & 0xFF,
                                                                    0x00, 0x00])

        motor_temp = msg.data[1]
        torque_current = (msg.data[2] | (msg.data[3] << 8)) * 0.01
        motor_speed = (msg.data[4] | (msg.data[5] << 8)) * 0.01
        motor_angle = (msg.data[6] | (msg.data[7] << 8)) * 0.01

        print(f"RX : motor temperature: {motor_temp}°C")
        print(f"RX : torque current: {torque_current}A")
        print(f"RX : motor speed: {motor_speed}dps")
        print(f"RX : motor angle: {motor_angle}°")

        return motor_temp, torque_current, motor_speed, motor_angle

