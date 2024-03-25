import unittest
import time
from MyActuatorRMD import MyActuatorRMD
from CanBusGsUsb import CanBusGsUsb

class TestActuatorMethods(unittest.TestCase):

    def setUp(self):
        can_bus=CanBusGsUsb(0,1000000)
        can_bus.setup()
        self.j1 = MyActuatorRMD.X.V3(1,1,can_bus)

    def test_pid_parameters(self):
        kp1, ki1, kp2, ki2, kp3, ki3 = self.j1.write_pid_parameter_to_RAM(110,80,5,5,20,50)
        print("Testing PID Parameters...", end=' ')
        if (kp1, ki1, kp2, ki2, kp3, ki3) == (110, 80, 5, 5, 20, 50):
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")

    def test_motor_acceleration(self):
        self.j1.read_acceleration()
        print("Testing Motor Acceleration...", end=' ')
        if self.j1.write_acceleration_to_RAM(200) == 200:
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")

    def test_motor_status_1(self):
        motor_temp, voltage, error_state = self.j1.read_motor_status_1_error_flag()
        print("Testing Motor Status 1...", end=' ')
        if motor_temp < 50 and 20 < voltage < 26 and error_state == 0x00:
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")

    def test_motor_status_2(self):
        motor_temp, torque_current, motor_speed, encoder_position = self.j1.read_motor_status_2()
        print("Testing Motor Status 2...", end=' ')
        if motor_temp < 50 and -2048 < torque_current < 2048 and 0 < encoder_position < 65535:
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")

    def test_motor_status_3(self):
        motor_temp, phase_A_current, phase_B_current, phase_C_current = self.j1.read_motor_status_3()
        print("Testing Motor Status 3...", end=' ')
        if motor_temp < 50:
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")

    def test_read_current_power_value(self):
        print("Testing Full Tests...")
        self.j1.read_current_power_value(debug=True)
        self.j1.read_system_runtime(debug=True)
        self.j1.read_system_software_version_date(debug=True)
        time.sleep(1)
        self.j1.close_system_brake(debug=True)
        time.sleep(3)
        self.j1.open_system_brake(debug=True)
        time.sleep(1)

    def test_speed_closed_loop_control(self):
        print("Testing Speed Closed Loop Control...", end=' ')
        self.j1.speed_closed_loop_control(360)  # 360°/s soit 1tr/s
        time.sleep(3)
        if self.j1.stop():
            print("\033[92m[OK]\033[0m")
        else:
            print("\033[91m[NOK]\033[0m")
        time.sleep(3)
        self.j1.motor_off()
        time.sleep(1)

    def test_write_multiturn_encoder_current_position_to_ROM_as_motor_zero(self):
        print("Testing Write Multiturn Encoder Current Position to ROM as Motor Zero...", end=' ')
        self.j1.write_multiturn_encoder_current_position_to_ROM_as_motor_zero()
        time.sleep(1)
        print("\033[92m[OK]\033[0m")

    def test_system_reset(self):
        print("Testing System Reset...", end=' ')
        self.j1.system_reset()
        time.sleep(3)
        print("\033[92m[OK]\033[0m")

    def test_absolute_position_closed_loop(self):
        print("Testing Absolute Position Closed Loop...", end=' ')
        self.j1.absolute_position_closed_loop_control(max_speed=360,         # 360°/s
                                                      target_position=0,     # 0° - not move
                                                      wait_for_completion=True,
                                                      debug=True)
        time.sleep(1)
        self.j1.absolute_position_closed_loop_control(max_speed=360,         # 360°/s
                                                      target_position=720,   # 720° - 2tr
                                                      wait_for_completion=True,
                                                      debug=True)
        time.sleep(1)
        self.j1.absolute_position_closed_loop_control(max_speed=360,         # 360°/s
                                                      target_position=-180,   # 720° - 2tr
                                                      wait_for_completion=True,
                                                      debug=True)
        time.sleep(1)
        self.j1.absolute_position_closed_loop_control(max_speed=180,         # 360°/s
                                                      target_position=720,   # 720° - 2tr
                                                      wait_for_completion=True,
                                                      debug=True)
        time.sleep(2)
        print("\033[92m[OK]\033[0m")

    def test_incremental_position_closed_loop(self):
        print("Testing Incremental Position Closed Loop...", end=' ')
        self.j1.incremental_position_closed_loop_control(max_speed=360,         # 360°/s
                                                         target_position=360,   # +360° - not move
                                                         wait_for_completion=True,
                                                         debug=True)
        time.sleep(1)
        self.j1.incremental_position_closed_loop_control(max_speed=360,         # 360°/s
                                                         target_position=720,   # +720° - 2tr
                                                         wait_for_completion=True,
                                                         debug=True)
        time.sleep(1)
        self.j1.incremental_position_closed_loop_control(max_speed=360,         # 360°/s
                                                         target_position=-720,  # -720° - 2tr
                                                         wait_for_completion=True,
                                                         debug=True)
        time.sleep(2)
        print("\033[92m[OK]\033[0m")

    def tearDown(self):
        self.j1.can_bus.shutdown()

if __name__ == '__main__':
    unittest.main()
