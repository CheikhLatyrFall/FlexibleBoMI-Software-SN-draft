#pragma once

#ifndef _USER_INTERFACE_H_
#define _USER_INTERFACE_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <ctime>
#include <time.h>

/*#include <iostream> 
#include <dislin.h>
#include <discpp.h>
#include "Defined_Macro.h"
#include "Control_HMI.h"*/

#include <termios.h>
#include <mySerial.h>
#include <libkindrv/kindrv.h>
#include <sys/select.h>
#include "types.h"
#include "exception.h"
#include "SDL/SDL.h"
#include "SDL/SDL_getenv.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "Dislin_Scope.h"
#include "Plot.h"
#include "Defined_Macro.h"

////////////////////////////////
#include <pthread.h>
#include <unistd.h>
#include "dataIO.h"
#include "classifiercollector.h"
#include "classifier_manager.h"
////////////////////////////////

using namespace KinDrv;
using namespace std;

namespace Control_IMU_JACO
{
	class User_Interface
	{
	public:
	
		//JACO instances
		bool JACO_STARTED; //OFF;
		bool JACO_CONNECTED ; //OFF;
		bool XBOX_CTRL_CONNECTED; // = ON;
		bool MOUSE_CONTROL_ENABLED; // = OFF;
		char CONTROL_MAPPING_MODE; // = 1;//2;//    1-->pitch,roll   2-->pitch,yaw
		bool JACO_CONTROL_ENABLED; // = OFF;
		int JACO_PREVIOUS_MODE; // = 0;
		bool FINGER_CONTROL; // = OFF;
		bool HEAD_MOTION_CONTROL; // = OFF;
		bool CONTROL_TRIGGER; // = SINGLE_CLICK;//HOLD_DOWN;//
		bool SAVE_DATA = OFF; //ON;//
		bool READ_CARTESIAN_POSITION; // = ON;//OFF;//
		int read_cart_pos_temp; // = 0;
		bool TCP_SERVER_SET; // = OFF;//ON;//
		bool RASPBIAN_CLIENT_SET; // = OFF;

		//internal use only
		bool SAVING_MOTION_DATA; //= OFF;//ON;// //FOR MONITORING AND ALGORITHM DESIGN. ONLY THE HEADSET IMU SENSOR SHOULD BE USED
		int MOTION_CLASS; // = 0;
		int NB_MOTION_CLASS; // = 13;//20;//
		int motion_data_index; // = 0;
		int motion_data_to_read; // = 600; // (1/62Hz)*500 ~ 8s

		bool SAVING_RANDOM_MOTION; // = OFF;//ON;//
		int RANDOM_MOTION; // = 0;
		int NB_RANDOM_MOTION; // = 29
		int random_data_to_read;// = 300;
		
		//USB
		mySerial serial1;
		bool PORT_COM_OUVERT;// = 0;
		const static int Packet_Size = 32;
		int PCKT [Packet_Size - 1];
		unsigned char data[Packet_Size]; //Raspbian
		const static int S_Of_F = 51;
		const static int E_Of_F = 52;
		bool Acq_Started;// = 0; //ON ou OFF
		unsigned char * flush_array = NULL; //Raspbian
		int BYTES_AVAILABLE;				//Raspbian
		int* pnbyrc = &BYTES_AVAILABLE;		//Raspbian
		int result; //Raspbian
		//////////////////////////////////////

		//NETWORK, SAFETY CONTROL - KEYPAD
		char EVENT_SAFETY_KEY;// = NO_EVENT;
		bool SAFETY_KEY_NETWORK;// = OFF;				//Indicates if the safetykey node is used. If ON, they safety relies on safetykey
		bool SAFETY_CONTROL_KEYPAD_TOUCH;// = ON;		//Indicates if the keypad or switch USB is used for safety control
		bool SAFETY_KEYPAD_TOUCH_PRESSED;// = OFF;		//Set when the safety keypad touch is pressed
		bool LONG_CONTRACTION_CTRL_ENABLE;// = OFF;
		//////////////////////////////////////

		//TIMING
		int refresh;// = 0;
		int elapsed;// = 20;                           //Indicates the time between screen refreshes (should be changed by using a timer)
		unsigned int start_time;// = 0;
		unsigned int stop_time;//  = 0;
		unsigned int time_print_console;// = 0;
		bool TIMER_STARTED;// = 0;
		//////////////////////////////////////

		//KEYPAD
		int in_kyp; char ch_kyp;// = 'A';
		bool kyp_ready;// = 1;
		bool kyp_temp;// = 1;
		int kyp_delay;// = 0;

		const static int nb_keys = 30;
		float keypressed [nb_keys];
		int return_kypad;
		int c;// = 0; //Raspbian keypress value
		int m;// = 0; //Raspbian mousepress value
		int flag;// = 0; //Raspbian
		SDL_Event kypevent;
		int quit;// = 0;
		//////////////////////////////////////

		//PRINT and PLOT
		Plot plot_setup;
		char PLOT_DATA;// = OFF;//PLOT_CTRL;//PLOT_MULTI_EMG;//
		bool PRINT;// = ON;
		bool PRINT_CALIB;// = OFF;
		//////////////////////////////////////

		//ALGO AND DATA
		Control_HMI CTRL_IMU;
		char EMG_PRECISION_MODE;// = EMG_16BIT_PRECISION;
		bool ControlJACO;// = 0; //ON ou OFF
		bool Bouton_Hold;// = 0; //ON ou OFF
		int mode_jaco_test;
		int test_timer_jaco;// = 0;
		bool api_rpi_set;// = OFF;
		//////////////////////////////////////

		// SAVING FILE
		ofstream output_file;
		char output_file_name[20];
		unsigned int file_time_stamp;// = 0;
		unsigned int file_time_stamp_start;// = 0;
		time_t current_date;
		struct tm *struct_date;
		//////////////////////////////////////

		// TCP Client
		int var_display;// = 0;
		int sockfd, portno, n;
		sockaddr_in serv_addr;
		hostent * server;
		unsigned char SendBuf[DEFAULT_BUFLEN];
		//////////////////////////////////////
		
		//jaco
		JacoArm *arm;
		jaco_joystick_t jojo_jaco;// = {0};
		jaco_position_t position_jaco;// = {0}; //36bytes
		/////////////////////////////////////
		
		User_Interface();
		~User_Interface();
		
		void initialize (void);
		void set (void);
		void routine (void);
		void update_console(void);
		int send_cmd_JACO(void);
		int check_keypad(void);
		void change_mode_emg(char value);
		int START_JACO_ARM(void);
		int STOP_JACO_ARM(void);
		int SET_JACO_ARM(void);
		
		void save_data_to_file(void);
		
		//internal use / algo design
		void save_motion_class_data(void);
		void generate_random_motion(void);

		void error(char * msg);
		int send_tcp_data_raspbian (unsigned char * data, int data_len);
		int init_tcp_client_com_raspbian ();
		
		void PrintKeyInfo(SDL_KeyboardEvent *key);
		void PrintModifiers(SDLMod mod);
		
		//jaco
		int goto_retract(JacoArm *arm);
		int goto_home(JacoArm *arm);

	};

}

#endif /* _USER_INTERFACE_H_ */
