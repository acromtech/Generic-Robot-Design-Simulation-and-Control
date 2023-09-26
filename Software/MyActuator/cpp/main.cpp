#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>

#include "can_wrapper.h"
#include "myactuator_wrapper.h"

int main(){
    Can can0;
    Joint joint1(can0,0);
    /*
    joint1.read_motor_status_1_error_flag();
    joint1.read_motor_status_2();
    joint1.read_motor_status_3();
    joint1.read_pid_parameter_to_RAM();
    joint1.read_acceleration();
    joint1.read_encoder_data();
    joint1.read_multiturn_encoder_position();
    joint1.read_singleturn_encoder_position();
    */
    joint1.read_acceleration();
    joint1.write_acceleration(2000);
    joint1.read_acceleration();
    while(1){
        joint1.write_position_multiturn_speed_closed_loop_control(8000,30000);
        sleep(8);
        joint1.write_position_multiturn_speed_closed_loop_control(8000,0);
        sleep(8);
    }
    
    
    return 0;
}
/*
void menu(){
    Joint mot1;
    unsigned int request;
    char test;

    std::cout << "MENU";
    std::cout << "2 : Change PID parameters to RAM";
    std::cout << "3 : Change PID parameters to ROM";
    std::cout << "5 : Change acceleration";
    std::cout << "7 : Change encoder offset";
    std::cout << "8 : Change current position to ROM as motor zero command";
    std::cout << "15 : motor off";
    std::cout << "16 : motor stop";
    std::cout << "17 : motor running";
    std::cout << "18 : torque closed-loop";
    std::cout << "19 : speed closed-loop";
    std::cout << "20 : Change position closed-loop parameters 1";
    std::cout << "21 : Change position closed-loop parameters 2";
    std::cout << "22 : Change position closed-loop parameters 3";
    std::cout << "23 : Change position closed-loop parameters 4";

    while(1){
        std::cout << "Select : ";
        std::cin >> request;
        std::cout << "What's motor id : ";
        std::cin >> mot1.id;

        switch(request){
            case 1:
                mot1.read_pid_parameter_to_RAM();
                break;
            case 2:
                std::cout << "What's motor current loop KP : ";
                std::cin >> mot1.current_loop_kp;
                std::cout << "What's motor current loop KI : ";
                std::cin >> mot1.current_loop_ki;
                std::cout << "What's motor speed loop KP : ";
                std::cin >> mot1.speed_loop_kp;
                std::cout << "What's motor speed loop KI : ";
                std::cin >> mot1.speed_loop_ki;
                std::cout << "What's motor position loop KP : ";
                std::cin >> mot1.position_loop_kp;
                std::cout << "What's motor position loop KI : ";
                std::cin >> mot1.position_loop_ki;
                mot1.write_pid_parameter_to_RAM();
                break;
            case 3:
                std::cout << "What's motor current loop KP : ";
                std::cin >> mot1.current_loop_kp;
                std::cout << "What's motor current loop KI : ";
                std::cin >> mot1.current_loop_ki;
                std::cout << "What's motor speed loop KP : ";
                std::cin >> mot1.speed_loop_kp;
                std::cout << "What's motor speed loop KI : ";
                std::cin >> mot1.speed_loop_ki;
                std::cout << "What's motor position loop KP : ";
                std::cin >> mot1.position_loop_kp;
                std::cout << "What's motor position loop KI : ";
                std::cin >> mot1.position_loop_ki;
                mot1.write_pid_parameter_to_ROM();
                break;
            case 4:
                mot1.read_acceleration();
                break;
            case 5:
                std::cout << "What's motor acceleration : ";
                std::cin >> mot1.acceleration;
                mot1.write_acceleration();
                break;
            case 6:
                mot1.read_encoder_data();
                break;
            case 7:
                std::cout << "What's motor encoder offset : ";
                std::cin >> mot1.encoder_offset;
                mot1.write_encoder_offset();
                break;
            case 8:
                mot1.write_current_position_to_ROM_as_motor_zero_position();
                break;
            case 9:
                mot1.read_multiturn_encoder_position();
                break;
            case 10:
                mot1.read_singleturn_encoder_position();
                break;
            case 11:
                mot1.read_motor_status_1_error_flag();
                break;
            case 12:
                mot1.read_clear_motor_error_flag();
                break;
            case 13:
                mot1.read_motor_status_2();
                break;
            case 14:
                mot1.read_motor_status_3();
                break;
            case 15:
                mot1.shutdown();
                break;
            case 16:
                mot1.stop();
                break;
            case 17:
                mot1.running();
                break;
            case 18:
                std::cout << "What's motor torque : ";
                std::cin >> mot1.torque;
                mot1.write_torque_closed_loop_control();
                break;
            case 19:
                std::cout << "What's motor speed : ";
                std::cin >> mot1.speed;
                mot1.write_speed_closed_loop_control();
                break;
            case 20:
                std::cout << "What's motor multiturn position : ";
                std::cin >> mot1.position;
                mot1.write_position_multiturn_closed_loop_control();
                break;
            case 21:
                std::cout << "What's motor multiturn speed : ";
                std::cin >> mot1.speed;
                std::cout << "What's motor multiturn position : ";
                std::cin >> mot1.position;
                mot1.write_position_multiturn_speed_closed_loop_control();
                break;
            case 22:
                std::cout << "What's motor direction (0=clockwise and 1=counterclockwise) : ";
                std::cin >> mot1.direction;
                std::cout << "What's motor multiturn position : ";
                std::cin >> mot1.position;
                mot1.write_position_singleturn_direction_closed_loop_control();
                break;
            case 23:
                std::cout << "What's motor direction (0=clockwise and 1=counterclockwise) : ";
                std::cin >> mot1.direction;
                std::cout << "What's motor multiturn speed : ";
                std::cin >> mot1.speed;
                std::cout << "What's motor multiturn position : ";
                std::cin >> mot1.position;
                mot1.write_position_singleturn_speed_direction_closed_loop_control();
                break;
            default:
                std::cout << "error incorrect input";
                break;
        }
        std::cout << "Continue ? [Y/N]";
        std::cin >> test;
        if(test=='N') return;
    }
}
*/