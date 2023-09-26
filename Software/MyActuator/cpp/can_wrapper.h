#ifndef CAN_WRAPPER_H
#define CAN_WRAPPER_H

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

#define TIMEOUT_READ_MS 3000

class Can{
	public :
		/**Attributes */
		int sock;
		
		/**Constructor */
		Can(){
			int ret;
			do{
				ret=Can::set_can_ON();
				sleep(2);
			}while(!ret);
		}
		
		/**Destructor */
		~Can(){
			set_can_OFF();
		}

		/**Methods */
		int send_frame(canid_t can_id,__u8 can_dlc,__u8 data[CAN_MAX_DLEN]){
			struct can_frame frame;
			memset(&frame,0,sizeof(struct can_frame));
			int nbytes;

			frame.can_id = can_id;
			frame.can_dlc = can_dlc;
			frame.data[0] = data[0];
			frame.data[1] = data[1];
			frame.data[2] = data[2];
			frame.data[3] = data[3];
			frame.data[4] = data[4];
			frame.data[5] = data[5];
			frame.data[6] = data[6];
			frame.data[7] = data[7];

			if(!(frame.can_id&CAN_EFF_FLAG)) printf("Transmit standard frame!\n");
			else printf("Transmit extended frame!\n");

			Can::display_frame(frame);

			nbytes=write(this->sock,&frame,sizeof(frame)); 
			if(nbytes!=sizeof(frame)){
				printf("Send frame incompletely!\r\n");
				system("sudo ip can0 down");
				return 0;
			}
			return 1;
		}
		
		can_frame receive_frame(canid_t id,__u8 addr){
			struct can_frame frame;
			clock_t s,f;
			memset(&frame,0,sizeof(struct can_frame));
			int nbytes;
			s=clock();
			do{
				nbytes=read(this->sock,&frame,sizeof(frame));
				if(nbytes>0){
					if(!(frame.can_id&CAN_EFF_FLAG)) printf("Received standard frame!\n");
					else printf("Received extended frame!\n");

					Can::display_frame(frame);
					
					f=clock();
					if(TIMEOUT_READ_MS-(f-s)<0){
						perror("\e[1;31mError\e[0;31m : Timeout\e[0m\n\r");
						exit(0);
					}
				}
			}while(frame.can_id==id && frame.data[0]==addr);
			return frame;
		}

		void display_frame(struct can_frame frame){
			printf("can_id  = 0x%X\r\n", frame.can_id);
			printf("can_dlc = %d\r\n", frame.can_dlc);
			for(int i=0;i<8;i++) printf("data[%d] = %d\r\n", i, frame.data[i]);
		}

		unsigned char find_can(const int port){
			char buf[128]={0};
			sprintf(buf,"/sys/class/net/can%d/can_bittiming/bitrate",port);
			return((access(buf,0)==0));
		}

		int set_can_ON(){
			int ret;
			struct sockaddr_can addr;
			struct ifreq ifr;
			if(system("sudo ip link set can0 down"))return 0;
			if(system("sudo ip link set can0 type can bitrate 1000000"))return 0;
			if(system("sudo ip link set can0 up"))return 0;
			/*Create socket*/
			this->sock=socket(PF_CAN, SOCK_RAW, CAN_RAW);
			if(this->sock<0){
				perror("\033[1;31Create socket PF_CAN failed!\033[0m");
				return 0;
			}
			std::cout << "Socket : OK" << std::endl;
			/*Specify can0 device*/
			strcpy(ifr.ifr_name, "can0");
			ret=ioctl(this->sock,SIOCGIFINDEX,&ifr);
			if(ret<0){
				perror("\033[1;31ioctl interface index failed!\033[0m");
				return 0;
			}
			/*Bind the socket to can0*/
			addr.can_family=PF_CAN;
			addr.can_ifindex=ifr.ifr_ifindex;
			ret=bind(this->sock,(struct sockaddr *)&addr,sizeof(addr));
			if(ret<0){
				perror("\033[1;31bind failed\033[0m");
				return 0;
			}
			std::cout << "Can openned" << std::endl;
			return 1;
		}

		int set_can_OFF(){
			close(this->sock);
			system("sudo ip link set can0 down");
			return 1;
		}
};
#endif