#ifndef MYACTUATOR_WRAPPER_H
#define MYACTUATOR_WRAPPER_H

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
#include <iostream>

#include "can_wrapper.h"

#define BASE_ADDR_ID 													0x141
#define NO_DLC															0x08

#define ADDR_SHUTDOWN 													0x80
#define ADDR_STOP 														0x81
#define ADDR_RUNNING													0x88
#define ADDR_BRAKE														0x78	
#define ADDR_CAN_ID_SETUP												0x79

/*WRITE ADDR*/
#define ADDR_WRITE_PID_PARAMETER_TO_RAM									0x31
#define ADDR_WRITE_PID_PARAMETER_TO_ROM									0x32
#define ADDR_WRITE_ACCELERATION_TO_RAM									0x34
#define ADDR_WRITE_ENCODER_OFFSET										0x91
#define ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION		0x19
#define ADDR_WRITE_TORQUE_CLOSED_LOOP_CONTROL							0xA1
#define ADDR_WRITE_SPEED_CLOSED_LOOP_CONTROL							0xA2
#define ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL				0xA3
#define ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL			0xA4
#define ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL				0xA5
#define ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL		0xA6

/*READ ADDR*/
#define ADDR_READ_PID_PARAMETER_TO_RAM									0x30
#define ADDR_READ_ACCELERATION											0x33
#define ADDR_READ_ENCODER_DATA											0x90
#define ADDR_READ_MULTITURN_ENCODER_POSITION							0x92
#define ADDR_READ_SINGLETURN_ENCODER_POSITION							0x94
#define ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG								0x9A
#define ADDR_READ_CLEAR_MOTOR_ERROR_FLAG								0x9B
#define ADDR_READ_MOTOR_STATUS_2										0x9C
#define ADDR_READ_MOTOR_STATUS_3										0x9D

/*Special address description flags for CAN_ID*/
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U

class Joint{
	private :
		/**Attributes */
		Can &can;
		__u8 current_loop_kp;
		__u8 current_loop_ki;
		__u8 speed_loop_kp;
		__u8 speed_loop_ki;
		__u8 position_loop_kp;
		__u8 position_loop_ki;
		__u8 acceleration; 
		__u8 encoder_offset;
		__u8 torque;
		__u8 speed;
		__u8 direction;
		__u8 position;
		__u8 id;

	public :
		/**Constructor */
		Joint(Can &c, unsigned int id) : can(c),id(id){
			this->id=id;
		}

		/**Destructor */
		~Joint(){

		}

		/**Methods */
		can_frame write_pid_parameter_to_RAM(__u8 current_loop_kp, __u8 current_loop_ki, __u8 speed_loop_kp, __u8 speed_loop_ki, __u8 position_loop_kp, __u8 position_loop_ki){
			__u8 TX_data[]={ADDR_WRITE_PID_PARAMETER_TO_RAM,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_PID_PARAMETER_TO_RAM);
			std::cout << "RX : current loop KP set to " 	<< RX_frame.data[2] << std::endl;
            std::cout << "RX : current loop KI set to " 	<< RX_frame.data[3] << std::endl;
            std::cout << "RX : speed loop KP set to "		<< RX_frame.data[4] << std::endl;
            std::cout << "RX : speed loop KI set to "		<< RX_frame.data[5] << std::endl;
            std::cout << "RX : position loop KP set to "	<< RX_frame.data[6] << std::endl;
            std::cout << "RX : position loop KI set to "	<< RX_frame.data[7] << std::endl;
			return RX_frame;
		}
		can_frame write_pid_parameter_to_ROM(__u8 current_loop_kp, __u8 current_loop_ki, __u8 speed_loop_kp, __u8 speed_loop_ki, __u8 position_loop_kp, __u8 position_loop_ki){
			__u8 TX_data[]={ADDR_WRITE_PID_PARAMETER_TO_ROM,0x00,current_loop_kp,current_loop_ki,speed_loop_kp,speed_loop_ki,position_loop_kp,position_loop_ki};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_PID_PARAMETER_TO_ROM);
			std::cout << "RX : current loop KP set to " 	<< RX_frame.data[2] << std::endl;
            std::cout << "RX : current loop KI set to " 	<< RX_frame.data[3] << std::endl;
            std::cout << "RX : speed loop KP set to "		<< RX_frame.data[4] << std::endl;
            std::cout << "RX : speed loop KI set to "		<< RX_frame.data[5] << std::endl;
            std::cout << "RX : position loop KP set to "	<< RX_frame.data[6] << std::endl;
            std::cout << "RX : position loop KI set to "	<< RX_frame.data[7] << std::endl;
			return RX_frame;
		}
		can_frame write_acceleration(int acceleration){
			__u8 TX_data[]={ADDR_WRITE_ACCELERATION_TO_RAM,0x00,0x00,0x00,(__u8)(acceleration&0xFF),(__u8)((acceleration&0xFF00)>>8),(__u8)((acceleration&0xFF0000)>>16),(__u8)((acceleration&0xFF000000)>>24)};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_ACCELERATION_TO_RAM);
			std::cout << "RX : acceleration set to " << (RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24) << "dps/s" << std::endl;
			return RX_frame;
		}
		can_frame write_encoder_offset(int offset){
			__u8 TX_data[]={ADDR_WRITE_ENCODER_OFFSET,0x00,0x00,0x00,0x00,0x00,(__u8)(offset&0xFF),(__u8)((offset&0xFF00)>>8)};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_ENCODER_OFFSET);
			std::cout << "RX : encoder offset set to " << ((RX_frame.data[6])+(RX_frame.data[7]<<8)) << std::endl;
			return RX_frame;
		}
		can_frame write_current_position_to_ROM_as_motor_zero_position(){
			__u8 TX_data[]={ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_CURRENT_POSITION_TO_ROM_AS_MOTOR_ZERO_POSITION);
			std::cout << "RX : initial position set to " << ((RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24)) << std::endl;
			return RX_frame;
		}
		can_frame write_torque_closed_loop_control(int torque){
			__u8 TX_data[]={ADDR_WRITE_TORQUE_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(__u8)((torque)&0xFF),(__u8)(((torque)&0xFF00)>>8),0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_TORQUE_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << RX_frame.data[1] << "°C" << std::endl;
            std::cout << "RX : torque current value (-2048~2048):" << ((RX_frame.data[2])+(RX_frame.data[3]<<8)) << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8)) << "dps" << std::endl;
            std::cout << "RX : motor position value (0~16383):" << ((RX_frame.data[6])+(RX_frame.data[7]<<8)) << std::endl;
			return RX_frame;
		}
		can_frame write_speed_closed_loop_control(int speed){
			__u8 TX_data[]={ADDR_WRITE_SPEED_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(__u8)((speed)&0xFF),(__u8)(((speed)&0xFF00)>>8),(__u8)(((speed)&0xFF0000)>>16),(__u8)(((speed)&0xFF000000)>>24)};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_SPEED_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : torque current value (-2048~2048):" << ((RX_frame.data[2])+(RX_frame.data[3]<<8))*1 << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8))*1 << "dps" << std::endl;
            std::cout << "RX : motor position value (0~16383):" << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*1 << std::endl;
			return RX_frame;
		}
		can_frame write_position_multiturn_closed_loop_control(int position){
			__u8 TX_data[]={ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL,0x00,0x00,0x00,(__u8)((position*100)&0xFF),(__u8)(((position*100)&0xFF00)>>8),(__u8)(((position*100)&0xFF0000)>>16),(__u8)(((position*100)&0xFF000000)>>24)};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_POSITION_MULTITURN_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : torque current:" << ((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01 << "A" << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01 << "dps" << std::endl;
            std::cout << "RX : motor angle:" << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" << std::endl;
			return RX_frame;
		}
		can_frame write_position_multiturn_speed_closed_loop_control(int speed, int position){
			__u8 TX_data[]={ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL,0x00,(__u8)((speed)&0xFF),(__u8)(((speed)&0xFF00)>>8),(__u8)((position*100)&0xFF),(__u8)(((position*100)&0xFF00)>>8),(__u8)(((position*100)&0xFF0000)>>16),(__u8)(((position*100)&0xFF000000)>>24)};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame;
			RX_frame=can.receive_frame(this->id,ADDR_WRITE_POSITION_MULTITURN_SPEED_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : torque current:" << ((RX_frame.data[3])+(RX_frame.data[2]<<8)) << "A" << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01 << "dps" << std::endl;
            std::cout << "RX : motor angle:" << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" << std::endl;
			return RX_frame;
		}
		can_frame write_position_singleturn_direction_closed_loop_control(int direction, int position){
			__u8 TX_data[]={ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL,(__u8)((direction)&0xFF),0x00,0x00,(__u8)((position*100)&0xFF),(__u8)(((position*100)&0xFF00)>>8),0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_SINGLETURN_DIRECTION_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : torque current:" << ((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01 << "A" << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01 << "dps" << std::endl;
            std::cout << "RX : motor angle:" << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" << std::endl;
			return RX_frame;
		}
		can_frame write_position_singleturn_speed_direction_closed_loop_control(int direction, int speed, int position){
			__u8 TX_data[]={ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL,(__u8)((direction)&0xFF),(__u8)((speed)&0xFF),(__u8)(((speed)&0xFF00)>>8),(__u8)((position*100)&0xFF),(__u8)(((position*100)&0xFF00)>>8),0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_WRITE_SINGLETURN_SPEED_DIRECTION_CLOSED_LOOP_CONTROL);
			std::cout << "RX : motor temperature:" << RX_frame.data[1] << "°C" << std::endl;
            std::cout << "RX : torque current:" << ((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01 << "A" << std::endl;
            std::cout << "RX : motor speed:" << ((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01 << "dps" << std::endl;
            std::cout << "RX : motor angle:" << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" << std::endl;
			return RX_frame;
		}
		can_frame read_pid_parameter_to_RAM(){
			__u8 TX_data[]={ADDR_READ_PID_PARAMETER_TO_RAM,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_PID_PARAMETER_TO_RAM);
			std::cout << "RX : current loop KP : " 	<< (RX_frame.data[2])*1 << std::endl;
            std::cout << "RX : current loop KI : " 	<< (RX_frame.data[3])*1 << std::endl;
            std::cout << "RX : speed loop KP : "	<< (RX_frame.data[4])*1 << std::endl;
            std::cout << "RX : speed loop KI : "	<< (RX_frame.data[5])*1 << std::endl;
            std::cout << "RX : position loop KP : "	<< (RX_frame.data[6])*1 << std::endl;
            std::cout << "RX : position loop KI : "	<< (RX_frame.data[7])*1 << std::endl;
			return RX_frame;
		}
		can_frame read_acceleration(){
			__u8 TX_data[]={ADDR_READ_ACCELERATION,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_ACCELERATION);
			std::cout << "RX : Acceleration : " << ((RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24))*1 << "dps/s" << std::endl;
			return RX_frame;
		}
		can_frame read_encoder_data(){
			__u8 TX_data[]={ADDR_READ_ENCODER_DATA,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_ENCODER_DATA);
			std::cout << "RX : motor position : " 			<< ((RX_frame.data[2])+(RX_frame.data[3]<<8))*1 << std::endl;
            std::cout << "RX : motor original position : " 	<< ((RX_frame.data[4])+(RX_frame.data[5]<<8))*1 << std::endl;
            std::cout << "RX : motor offset : "				<< ((RX_frame.data[6])+(RX_frame.data[7]<<8))*1 << std::endl;
			return RX_frame;
		}
		can_frame read_multiturn_encoder_position(){
			__u8 TX_data[]={ADDR_READ_MULTITURN_ENCODER_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_MULTITURN_ENCODER_POSITION);
			std::cout << "RX : Encoder multiturn position : " << ((RX_frame.data[1])*0,01+(RX_frame.data[2]<<8)*0,01+(RX_frame.data[3]<<16)*0,01+(RX_frame.data[4]<<24)*0,01+(RX_frame.data[5]<<32)*0,01+((int64_t)RX_frame.data[6]<<40)*0,01+(RX_frame.data[7]<<48)) << "°" << std::endl;
			//std::cout << "RX : Encoder multiturn position : " << ((RX_frame.data[1] | RX_frame.data[2] | RX_frame.data[3] | RX_frame.data[4]<<24 | RX_frame.data[5] | RX_frame.data[6]<<40 | RX_frame.data[7])*0,01) << "°" << std::endl;
			return RX_frame;
		}
		can_frame read_singleturn_encoder_position(){
			__u8 TX_data[]={ADDR_READ_SINGLETURN_ENCODER_POSITION,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_SINGLETURN_ENCODER_POSITION);
			std::cout << "RX :Encoder singleturn position : " << ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" << std::endl;
			return RX_frame;
		}
		can_frame read_motor_status_1_error_flag(){
			__u8 TX_data[]={ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_MOTOR_STATUS_1_ERROR_FLAG);
			std::cout << "Joint temperature :" << (RX_frame.data[1])*1 << "°C" << std::endl;
			std::cout << "Joint voltage :" << ((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1 << "V" << std::endl;
			if(RX_frame.data[7]==0x00) 		std::cout << "\033[1;32mVoltage status: OK\033[0m"						<<	"\n\033[1;32mTemperature status: OK\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x01) std::cout << "\033[1;34mVoltage status: low voltage protection\033[0m"	<<	"\n\033[1;32mTemperature status: OK\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x08) std::cout << "\033[1;32mVoltage status: OK\033[0m"						<<	"\n\033[1;34mTemperature status: over temperature protection\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x09) std::cout << "\033[1;34mVoltage status: low voltage protection\033[0m"	<<	"\n\033[1;34mTemperature status: over temperature protection\033[0m" << std::endl;
			else 							std::cout << "\033[1;31mError : unknown number\033[0m" << std::endl;
			return RX_frame;
		}
		can_frame read_clear_motor_error_flag(){
			__u8 TX_data[]={ADDR_READ_CLEAR_MOTOR_ERROR_FLAG,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_CLEAR_MOTOR_ERROR_FLAG);
			std::cout << "Joint temperature :" << (RX_frame.data[1])*1 << "°C" << std::endl;
			std::cout << "Joint voltage :" << ((RX_frame.data[3])+(RX_frame.data[4]<<8))*0.1 << "V" << std::endl;
			if(RX_frame.data[7]==0x00) 		std::cout << "\033[1;32mVoltage status: OK\033[0m"						<<	"\n\033[1;32mTemperature status: OK\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x01) std::cout << "\033[1;34mVoltage status: low voltage protection\033[0m"	<<	"\n\033[1;32mTemperature status: OK\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x08) std::cout << "\033[1;32mVoltage status: OK\033[0m"						<<	"\n\033[1;34mTemperature status: over temperature protection\033[0m" << std::endl;
			else if(RX_frame.data[7]==0x09) std::cout << "\033[1;34mVoltage status: low voltage protection\033[0m"	<<	"\n\033[1;34mTemperature status: over temperature protection\033[0m" << std::endl;
			else 							std::cout << "\033[1;31mError : unknown number\033[0m" << std::endl;
			return RX_frame;
		}
		can_frame read_motor_status_2(){
			__u8 TX_data[]={ADDR_READ_MOTOR_STATUS_2,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_MOTOR_STATUS_2);
			std::cout << "RX : motor temperature:" 	<< (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : torque current:" 	<< ((RX_frame.data[2])+(RX_frame.data[3]<<8))*0.01 << "A" 	<< std::endl;
            std::cout << "RX : motor speed:" 		<< ((RX_frame.data[4])+(RX_frame.data[5]<<8))*0.01 << "dps" << std::endl;
            std::cout << "RX : motor angle:" 		<< ((RX_frame.data[6])+(RX_frame.data[7]<<8))*0.01 << "°" 	<< std::endl;
			return RX_frame;
		}
		can_frame read_motor_status_3(){
			__u8 TX_data[]={ADDR_READ_MOTOR_STATUS_3,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_READ_MOTOR_STATUS_3);
			std::cout << "RX : motor temperature:" 	<< (RX_frame.data[1])*1 << "°C" << std::endl;
            std::cout << "RX : Phase A current:" 	<< ((RX_frame.data[2])+(RX_frame.data[3]<<8))/64 << "A" << std::endl;
            std::cout << "RX : Phase B current:" 	<< ((RX_frame.data[4])+(RX_frame.data[5]<<8))/64 << "A" << std::endl;
            std::cout << "RX : Phase C current:" 	<< ((RX_frame.data[6])+(RX_frame.data[7]<<8))/64 << "A" << std::endl;
			return RX_frame;
		}
		can_frame shutdown(){
			__u8 TX_data[]={ADDR_SHUTDOWN,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_SHUTDOWN);
			std::cout << "RX : Shutdown : OK" << std::endl;
			return RX_frame;
		}
		can_frame stop(){
			__u8 TX_data[]={ADDR_STOP,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_STOP);
			std::cout << "RX : Stop : OK" << std::endl;
			return RX_frame;
		}
		can_frame running(){
			__u8 TX_data[]={ADDR_RUNNING,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_RUNNING);
			std::cout << "RX : Running : OK" << std::endl;
			return RX_frame;
		}
		can_frame brake(){
			__u8 TX_data[]={ADDR_BRAKE,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_BRAKE);
			std::cout << "RX : Brake : OK" << std::endl;
			return RX_frame;
		}
		can_frame CAN_ID_setup(){
			__u8 TX_data[]={ADDR_CAN_ID_SETUP,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
			can.send_frame(BASE_ADDR_ID+this->id,NO_DLC,TX_data);
			can_frame RX_frame=can.receive_frame(this->id,ADDR_CAN_ID_SETUP);
			std::cout << "RX : Read and write flag : " << (RX_frame.data[2])*1 << std::endl;
			std::cout << "RX : CAN ID setup :" 		<< ((RX_frame.data[4])+(RX_frame.data[5]<<8)+(RX_frame.data[6]<<16)+(RX_frame.data[7]<<24))*0.01 << "°" 	<< std::endl;
			return RX_frame;
		}
		int display_parameters();
};

#endif