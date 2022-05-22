
/***************************************************************************
 *  main.cpp - Projet de contrôle inertiel - Tristan
 *  Created: Tues July 25 2017
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <string>

#include <ctime>
#include <termios.h>

#include "Defined_Macro.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"
#include "Control_JACO.h"
#include "Dislin_Scope.h"
#include "exception.h"

#include <dislin.h>
#include <discpp.h>

#include <libkindrv/kindrv.h>
#include <sys/select.h>
#include "test_print.h"
#include "mySerial.h"

using namespace Control_IMU_JACO;
using namespace KinDrv;
using namespace std;

void update_console(void);
void update_plot (void);
int check_keypad(void);
int goto_home(JacoArm *arm);
void test_libkindrv (void);
void init_API (void);
void algo_tristan (void);

struct termios orig_termios;
void reset_terminal_mode ();
void set_conio_terminal_mode ();
int kbhit ();
int getch ();
int result;
const char* usb_dev;
std::string usb_name;
int flag=0;
int c = 0;

bool PORT_COM_OUVERT = 0;
const int Packet_Size = 32;
int PCKT [Packet_Size - 1];
unsigned char  data[33];
static int S_Of_F = 51;
static int E_Of_F = 52;
int JACO_PREVIOUS_MODE = 0;
static bool ControlJACO = 0; //ON ou OFF
static bool Acq_Started = 0; //ON ou OFF
static bool Bouton_Hold = 0; //ON ou OFF
char EVENT_SAFETY_KEY = NO_EVENT;
/////
unsigned char INIT_KY[256];
/////
int refresh = 0;
int elapsed = 10;

//KEYPAD
int in_kyp; char ch_kyp;
bool kyp_ready = 0;
int kyp_delay = 0;
const int nb_keys = 23;
float keypressed [nb_keys];
int return_kypad;

//PRINTING
bool PRINT = ON;
bool PRINT_CALIB = ON;

//PLOT
bool PLOT_DATA = OFF;

const int NB_SAMPLES_BY_PCKT = 9;
const int NB_SAMPLES_TO_PLOT = 11 * NB_SAMPLES_BY_PCKT;
const int NB_SCOPE_SAMPLES = 50 * NB_SAMPLES_TO_PLOT;

const int NB_TKE_TO_PLOT = 15;
const int NB_SCOPE_TKE = 50 * NB_TKE_TO_PLOT;

const int NB_IMU_TO_PLOT = 5;//2;//
const int NB_SCOPE_IMU = 50 * NB_IMU_TO_PLOT;// 10 * NB_IMU_TO_PLOT; //

Dislin_Scope PLOT_EMG;
float new_emg[NB_SAMPLES_TO_PLOT]; int i_nemg = 0;
float new2_emg[NB_SAMPLES_TO_PLOT]; int i_n2emg = 0;
Dislin_Scope PLOT_TKE;
float new_tke[NB_TKE_TO_PLOT]; int i_ntke = 0;
float th = 0;
Dislin_Scope PLOT_IMU;
float new_imu[4];

Control_JACO CTRL_IMU;

int mode_jaco_test;
int test_timer_jaco = 0;
unsigned int start_time;
unsigned int stop_time;
int BYTES_AVAILABLE;
int* pnbyrc = &BYTES_AVAILABLE;

bool TIMER_STARTED = 0;
bool JACO_STARTED = 0;
bool JACO_CONNECTED = 1;

unsigned char * flush_array = NULL;

JacoArm *arm;
jaco_joystick_t jojo_jaco = {0};

int main()
{
  printf(" ____     ___  ____  _____    _____  _____  _____  ____  ____  _____ _____ \n");
  printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
  printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
  printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
  printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
  printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

  if (JACO_CONNECTED)
  {
	init_API();
	/////  I  N  I  T       J  A  C  O       T  E  S  T        K  I  N  D  R  V   //////
	//test_libkindrv ();
	////////////////////////////////////////////////////////////////////////////////////
	//return 0;
  }

  CTRL_IMU.initialize();

  if (PLOT_DATA)
  {
		PLOT_EMG.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2);
		PLOT_TKE.initialize(TKE_SCOPE, 2, NB_SCOPE_TKE, NB_TKE_TO_PLOT, 2);
		PLOT_IMU.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU, NB_IMU_TO_PLOT, 2);
  }

  float FREQ;
  int i;

  ///////////////////////////////////////
  result=system("ls -l /dev/ttyUSB*");
  if (result != 0)
  {
	std::cout << "\n no USB devices were found\n" << std::endl;
	return 0;
  }
  else
  {
	//printf("\nEnter /dev/ttyUSB* wished \n");
	//std::cin >> usb_name;
	//usb_dev = usb_name.c_str();
	usb_dev = "/dev/ttyUSB0";
  }
	mySerial serial1(usb_dev, 1998848);//115200;//);
  
	while (1)
	{
	
	//read keypad
	set_conio_terminal_mode();
	if (kbhit())
	{
		c=getch();
	}
	reset_terminal_mode();

	//read data
	data[0] = 0;
	while(data[0] != 0x33)
	{
		serial1.Receive(data,1);
	}
	refresh ++;
	serial1.NumberByteRcv(*pnbyrc);
	
	if(CTRL_IMU.calDone >= 100 && CTRL_IMU.calDone != 12345){
	flush_array = new unsigned char [BYTES_AVAILABLE];
	serial1.Receive(flush_array, BYTES_AVAILABLE);
	}

	serial1.Receive(data,Packet_Size-1);//read all packet
	if (data[Packet_Size-2] == 0x34)
	{
		for (int kl = 0; kl < Packet_Size-1; kl++)
		{
			PCKT[kl] = (int) data[kl];
		}
		CTRL_IMU.process_payload(PCKT);

		//TEST TRISTHEAD_IMU.yawctrlAN57.578888,
		return_kypad = check_keypad();
	}
  
  
///////////		Calibration 	//////////////
	if (CTRL_IMU.calDone <= 100) //First wprintf("Nb Bytes Received : %d\n\n", *pnbyrc);e run only this if() --> Nominal calibration
	{
		/*if (CTRL_IMU.HEAD_IMU.yawctrl < 0){
			CTRL_IMU.HEAD_IMU.yawctrl = 360 + CTRL_IMU.HEAD_IMU.yawctrl;
		} else if (CTRL_IMU.HEAD_IMU.yawctrl > 0){
			CTRL_IMU.HEAD_IMU.yawctrl = CTRL_IMU.HEAD_IMU.yawctrl;
		}*/
		
		//Initial pitch and yaw values
		CTRL_IMU.HEAD_IMU.pitchoffset = CTRL_IMU.HEAD_IMU.pitchraw;
		CTRL_IMU.HEAD_IMU.yawoffset = CTRL_IMU.HEAD_IMU.yawraw;
		CTRL_IMU.calValues[0] = CTRL_IMU.HEAD_IMU.pitchraw;
		CTRL_IMU.calValues[1] = -CTRL_IMU.HEAD_IMU.yawraw;
	
		
		//Polar coordinates go to array5HEAD_IMU.pitchraw - calValues[0]7.578888,
		CTRL_IMU.polarCalValues[0] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[0],CTRL_IMU.calValues[1]);
		CTRL_IMU.polarCalValues[1] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[0],CTRL_IMU.calValues[1]);
		
		system("clear");
		printf("Neutral calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[0], CTRL_IMU.calValues[1]);
		printf("Neutral calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[0], CTRL_IMU.polarCalValues[1]);

		CTRL_IMU.calDone ++;
	}
	
	/////////////////Calibration min./max.
	if(CTRL_IMU.calDone > 100)
	{
	if(CTRL_IMU.calDone > 100 && CTRL_IMU.calDone != 12345)
	{
		printf("Press V to save X_Max values\n");
		}
	while(CTRL_IMU.calDone != 12345){
		flush_array = new unsigned char [BYTES_AVAILABLE];
		serial1.Receive(flush_array, BYTES_AVAILABLE);
		
		
		int num_kyp = 0;
		if (kyp_delay != 0)
		{
		kyp_delay++;
		if (kyp_delay == 5)
			kyp_delay = 0;
		}
		
		//read keypad
		set_conio_terminal_mode();
		if (kbhit())
		{
			c=getch();
		}
		reset_terminal_mode();
		
		while (c == V_KY && !kyp_delay) //Press V to save X_Max values
		{
		c = 0;
		
		for(int i = 0; i <= 15; i++){
		//read data
		data[0] = 0;
		while(data[0] != 0x33)
		{
			serial1.Receive(data,1);
		}
		refresh ++;

		serial1.Receive(data,Packet_Size-1);//read all packet
		if (data[Packet_Size-2] == 0x34)
		{
			for (int kl = 0; kl < Packet_Size-1; kl++)
			{
				PCKT[kl] = (int) data[kl];
			}
			CTRL_IMU.process_payload(PCKT);

			//TEST TRISTAN57.578888,
			return_kypad = check_keypad();
		}	
			
		//Initial pitch and yaw values
		CTRL_IMU.calValues[2] = 0;//CTRL_IMU.HEAD_IMU.pitchctrl;//CTRL_IMU.HEAD_IMU.pitchraw - CTRL_IMU.calValues[0];
		CTRL_IMU.calValues[3] = 40;//-CTRL_IMU.HEAD_IMU.yawctrl;//-(CTRL_IMU.HEAD_IMU.yawraw - CTRL_IMU.calValues[1]);
		
		//Polar coordinates go to array
		CTRL_IMU.polarCalValues[2] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[2],CTRL_IMU.calValues[3]);
		CTRL_IMU.polarCalValues[3] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[2],CTRL_IMU.calValues[3]);
		
		system("clear");
		printf("X-Axis Max. calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[2], CTRL_IMU.calValues[3]);
		printf("X-Axis Max. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[2], CTRL_IMU.polarCalValues[3]);
		printf("Press E to save X_Min values\n");
		}
		}
		
		while (c == E_KY && !kyp_delay) //Press E to save X_Min values
		{
		c = 0;
		for(int i = 0; i <= 15; i++){
		//read data
		data[0] = 0;
		while(data[0] != 0x33)
		{
			serial1.Receive(data,1);
		}
		refresh ++;

		serial1.Receive(data,Packet_Size-1);//read all packet
		if (data[Packet_Size-2] == 0x34)
		{
			for (int kl = 0; kl < Packet_Size-1; kl++)
			{
				PCKT[kl] = (int) data[kl];
			}
			CTRL_IMU.process_payload(PCKT);

			//TEST TRISTAN57.578888,
			return_kypad = check_keypad();
		}
			
		//Initial pitch and yinaw values
		CTRL_IMU.calValues[4] = 0;//CTRL_IMU.HEAD_IMU.pitchctrl;//CTRL_IMU.HEAD_IMU.pitchraw - CTRL_IMU.calValues[0];
		CTRL_IMU.calValues[5] = -40;//-CTRL_IMU.HEAD_IMU.yawctrl;//-(CTRL_IMU.HEAD_IMU.yawraw - CTRL_IMU.calValues[1]);
		
		//Polar coordinates go to array
		CTRL_IMU.polarCalValues[4] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[4],CTRL_IMU.calValues[5]);
		CTRL_IMU.polarCalValues[5] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[4],CTRL_IMU.calValues[5]);
		
		system("clear");
		printf("X-Axis Min. calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[4], CTRL_IMU.calValues[5]);
		printf("X-Axis Min. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[4], CTRL_IMU.polarCalValues[5]);
		printf("Press D to save Y_Max values\n");
		}
		}
		
		while (c == D_KY && !kyp_delay) //Press D to save Y_Max values
		{
		c = 0;
		for(int i = 0; i <= 15; i++){
		//read data
		data[0] = 0;
		while(data[0] != 0x33)
		{
			serial1.Receive(data,1);
		}
		refresh ++;

		serial1.Receive(data,Packet_Size-1);//read all packet
		if (data[Packet_Size-2] == 0x34)
		{
			for (int kl = 0; kl < Packet_Size-1; kl++)
			{
				PCKT[kl] = (int) data[kl];
			}
			CTRL_IMU.process_payload(PCKT);

			//TEST TRISTAN
			return_kypad = check_keypad();
		}
				
		//Initial pitch and yaw values
		CTRL_IMU.calValues[6] = 180;//CTRL_IMU.HEAD_IMU.pitchctrl;//CTRL_IMU.HEAD_IMU.pitchraw - CTRL_IMU.calValues[0];
		CTRL_IMU.calValues[7] = 0;//-CTRL_IMU.HEAD_IMU.yawctrl;//-(CTRL_IMU.HEAD_IMU.yawraw - CTRL_IMU.calValues[1]);
		
		//Polar coordinates go to array
		CTRL_IMU.polarCalValues[6] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[6],CTRL_IMU.calValues[7]);
		CTRL_IMU.polarCalValues[7] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[6],CTRL_IMU.calValues[7]);
		
		system("clear");
		printf("Y-Axis Max. calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[6], CTRL_IMU.calValues[7]);
		printf("Y-Axis Max. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[6], CTRL_IMU.polarCalValues[7]);
		printf("Press F to save Y_Min values\n");
		}
		}
		
		while (c == F_KY && !kyp_delay) //Press F to save Y_Min values
		{
		c = 0;
		for(int i = 0; i <= 15; i++){
		//read data
		data[0] = 0;
		while(data[0] != 0x33)
		{
			serial1.Receive(data,1);
		}
		refresh ++;

		serial1.Receive(data,Packet_Size-1);//read all packet
		if (data[Packet_Size-2] == 0x34)
		{
			for (int kl = 0; kl < Packet_Size-1; kl++)
			{
				PCKT[kl] = (int) data[kl];
			}
			CTRL_IMU.process_payload(PCKT);

			//TEST TRISTAN57.578888,
			return_kypad = check_keypad();
		}
				
		//Initial pitch and yaw values
		CTRL_IMU.calValues[8] = -180;//CTRL_IMU.HEAD_IMU.pitchctrl;//CTRL_IMU.HEAD_IMU.pitchraw - CTRL_IMU.calValues[0];
		CTRL_IMU.calValues[9] = 0;//-CTRL_IMU.HEAD_IMU.yawctrl;//-(CTRL_IMU.HEAD_IMU.yawraw - CTRL_IMU.calValues[1]);
	
		
		//Polar coordinates go to array
		CTRL_IMU.polarCalValues[8] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[8],CTRL_IMU.calValues[9]);
		CTRL_IMU.polarCalValues[9] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[8],CTRL_IMU.calValues[9]);
		
		system("clear");
		printf("Y-Axis Min. calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[8], CTRL_IMU.calValues[9]);
		printf("Y-Axis Min. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[8], CTRL_IMU.polarCalValues[9]);
		printf("All calibration done\n");
		}
		CTRL_IMU.calDone = 12345; //Exit calibration loop after calibration done
		}
	}
	////while(1) normale
  	update_console();
	CTRL_IMU.finger_ctrl();
	
	if (JACO_CONNECTED)
		{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}
		
	if (JACO_CONNECTED)
		{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}
	
	}
	}
}

/*void algo_tristan (void)
{
	/331.217560///////////		Calibration 	//////////////
	if (CTRL_IMU.calDone <= 100) //First we run only this if() --> Nominal calibration
	{
		if (CTRL_IMU.HEAD_IMU.yawctrl < 0){CTRL_IMU.HEAD_IMU.yawctrl
			CTRL_IMU.HEAD_IMU.yawctrl = 360 + CTRL_IMU.HEAD_IMU.yawctrl;
		} else if (CTRL_IMU.HEAD_IMU.yawctrl > 0){
			CTRL_IMU.HEAD_IMU.yawctrl = CTRL_IMU.HEAD_IMU.yawctrl;
		}
		
		//Initial pitch and yaw values
		CTRL_IMU.calValues[0] = CTRL_IMU.HEAD_IMU.pitchraw;
		CTRL_IMU.calValues[1] = CTRL_IMU.HEAD_IMU.yawctrl;
	CTRL_IMU.Convert2PolaCTRL_IMU.HEAD_IMU.pitchrawrAmplitude(CTRL_IMU.calValues[0],CTRL_IMU.calValues[1])
		
		//Polar coordinates go to array
		CTRL_IMU.polarCalValues[0] = CTRL_IMU.Convert2PolarAmplitude(CTRL_IMU.calValues[0],CTRL_IMU.calValues[1]);
		CTRL_IMU.polarCalValues[1] = CTRL_IMU.Convert2PolarTheta(CTRL_IMU.calValues[0],CTRL_IMU.calValues[1]);
		
		system("clear");
		printf("Neutral calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[0], CTRL_IMU.calValues[0]);
		printf("Neutral calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[0], CTRL_IMU.polarCalValues[0]);

	if (JACO_CONNECTED)
		{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}
		
	if (JACO_CONNECTED)
		{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}CTRL_IMU.calDone ++;
	}

	if (CTRL_IMU.calDone > 100 && CTRL_IMU.calDone <= 500){ //Then we run this second if() --> Min./Max. calibration
		
		if ((CTRL_IMU.calDone % 3) == 0){
		system("clear");
		printf("Nominal Calibration Done\n");
		printf("calDone Value: %d\n", CTRL_IMU.calDone);
		printf("Y-Axis Min./Max. Calibration Values: %f,\t%f\n", CTRL_IMU.calValues[4], CTRL_IMU.calValues[3]);
		printf("X-Axis Min./Max. Calibration Values: %f,\t%f\n", CTRL_IMU.calValues[2], CTRL_IMU.calValues[1]);
		}
		
		CTRL_IMU.calDone ++;
	}
SPACEBAR_KY
	if(CTRL_IMU.calDone > 500)
	{	//usb_dev = usb_name.c_str();
	usb_dev = "/dev/ttyUSB0";
  }
	mySerial serial1(usb_dev, 1998848);//115200;//);
}
HEAD_IMU.yawctrl
  while (1)
  {
	//read keypad
	set_conio_terminal_mode();
	if (kbhit())
	{
		c=getch();
	}
	reset_terminal_mode();

	//read data
	data[0] = 0;
	while(data[0] != 0x33)		printf("Neutral calibration pitch and roll: %f,\t%f\n", CTRL_IMU.calValues[0], CTRL_IMU.calValues[1]);
		printf("Neutral calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[0], CTRL_IMU.polarCalValues[1]);
	{
		serial1.Receive(data,1);
	}
	r331.217560efresh ++;

	serial1.Receive(data,Packet_Size-1);//read all packet
	if (data[Packet_Size-2] == 0x34)
	{
		for (int kl = 0; kl < Packet_Size-1; kl++)
		{
			PCKT[kl] = (int) data[kl];
		}
		CTRL_IMU.process_payload(PCKT);
CTRL_IMU.HEAD_IMU.yawctrl
		//TEST TRISTAN
		return_kypad = check_keypad();
	}
	
	if(CTRL_IMU.calDone > 500)
	{	//usb_dev = usb_name.c_str();
	usb_dev = "/dev/ttyUSB0";
  }
	mySerial serial1(usb_dev, 1998848);//115200;//);

		update_plot();
		//printf("Normal while(1)");
		update_console ();
		if (TIMER_STARTED)
			stop_time = clock() - start_time;
			
			
		if (JACO_CONNECTED)
		{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}
	}	
	if (JACO_CONNECTED)
		SPACEBAR_KY{
			//////////////		 JACO Command 		//////////////
			jojo_jaco.axis.trans_lr = CTRL_IMU.Tristan_CmdX_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			jojo_jaco.axis.trans_rot = CTRL_IMU.Tristan_CmdY_new; //[-2.5,2.5] --> [-20cm/s, 20cm/s]
			arm->move_joystick_axis(jojo_jaco.axis);
			//jojo_jaco.axis.trans_lr = 0.0;
			//jojo_jaco.axis.trans_rot = 0.0;
			//printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			//printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
		}
}*/

void update_console(void)
{
#ifdef _WIN32
	if (START_TIME != 0)
		END_TIME = omp_get_wtime() - START_TIME;
	START_TIME = omp_get_wtime();
#endif
	if (refresh >= elapsed)
	{
		if (PRINT)
		{
#ifdef _WIN32
			system("cls");
#else
			system("clear");
#endif
			printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
			printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
			printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
			printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
			printf(" _/  |/  / | || |__ | | | |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
			printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

			printf("___________________________________________________________________________\n");

			//if (c != 0)
			//	printf("keypad pressed = %d\n",c);
			//c = 0;
			if (return_kypad != 0)
				printf("keypad pressed = %d\n",return_kypad);
			printf("LIMIT MISSING = %d\n", (int)CTRL_IMU.limit_missing);
			printf("GETCH c = %d\n", c);
			printf("Time Interval in millisecs : %d\n",stop_time);
			printf("Nb Bytes Received : %d\n\n", *pnbyrc);

			if (CTRL_IMU.calDone)		
			printf("Neutral calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[0], CTRL_IMU.polarCalValues[1]);
			printf("X-Axis Max. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[2], CTRL_IMU.polarCalValues[3]);
			printf("X-Axis Min. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[4], CTRL_IMU.polarCalValues[5]);
			printf("Y-Axis Max. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[6], CTRL_IMU.polarCalValues[7]);
			printf("Y-Axis Min. calibration amplitude and theta: %f,\t%f\n", CTRL_IMU.polarCalValues[8], CTRL_IMU.polarCalValues[9]);
			printf("___________________________________________________________________________\n");
			printf("Polar amplitude Tristan: %f\n", CTRL_IMU.amp_Tristan);
			printf("Polar theta value Tristan: %f\n", CTRL_IMU.theta_Tristan);
			printf("___________________________________________________________________________\n");

			if (ControlJACO == 1)
				printf("API SET\n");
			else
				printf("API CLOSED\n");

			if (Bouton_Hold == 1)
				printf("CONTROL SET\n");
			else
				printf("CONTROL STOPPED\n");

			if (EVENT_SAFETY_KEY == ENTER_STANDBY)
				printf("SYSTEM IN STANDBY MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CALIBRATION)
				printf("SYSTEM IN CALIBRATION MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CONTROL)
				printf("SYSTEM IN 9CONTROL MODE\n");

			printf("variable test routine : %d\n", test_timer_jaco);

			printf("***************************************\n");
			if (CTRL_IMU.HEAD_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ HEADSET = %d Hz\n", CTRL_IMU.HEAD_IMU.FREQ);
				printf("RAW  : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.pitch_TR, (int)CTRL_IMU.HEAD_IMU.rollraw, (int)CTRL_IMU.yaw_TR);
				printf("CTRL : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchctrl, (int)CTRL_IMU.HEAD_IMU.rollctrl, (int)-CTRL_IMU.HEAD_IMU.yawctrl);
				printf("     : AMP   = %f __ THETA = %f | [deg]\n", (float)CTRL_IMU.Amp, (float)CTRL_IMU.Theta);
				if (CTRL_IMU.IMU_CALIBRATED)
					printf("HEADSET CALIBRATED\n");
				else
					printf("HEADSET NOT CALIBRATED\n");
			}
			else
				printf("HEADSET SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ CHAIR  = %d Hz\n", CTRL_IMU.CHAIR_IMU.FREQ);
				printf("HEADING JACO  : %d [deg]\n", (int)CTRL_IMU.CHAIR_IMU.yawraw);
				printf("CHAIR VELOCITY = %d deg/s   |  THRESHOLD = %d deg/s\n", (int)CTRL_IMU.CHAIR_IMU.MeanVelYaw, (int)CTRL_IMU.CHAIR_IMU.ThRotReference);
			}
			else
				printf("REFERENCE SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.EMG_1_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ EMG  = %d Hz\n", CTRL_IMU.EMG_1.FREQ * 9);
				printf("CURRENT TKE  = %f  |  THRESHOLD TKE = %f\n", CTRL_IMU.EMG_1.MEAN_TKE_CURRENT, CTRL_IMU.EMG_1.THRESHOLD);
				printf("UPPER THRES. = %f  |  LOWER THRES   = %f\n", CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD);
				printf("DETECTION EMG = %d |  CONTRACTION DURATION = %d\n ", CTRL_IMU.EMG_1.EMG_DETECTED, CTRL_IMU.EMG_1.time_contraction);
				if (CTRL_IMU.EMG_CALIBRATED)
					printf("EMG CALIBRATED\n");
				else
					printf("EMG NOT CALIBRATED\n");
			}
			else
				printf("EMG SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			//printf("PARAMS : Ctrl Z = %d | Ref active = %d | TH ROT CHAIR = %d deg/s (window = %d)\n", CTRL_IMU.ZActive, CTRL_IMU.ReferenceActive, (int)CTRL_IMU.CHAIR_IMU.ThRotReference, CTRL_IMU.CHAIR_IMU.nb_val_gyr);
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
				printf("HEADING OF9FSET = %d\n", (int)CTRL_IMU.Theta_offset);
			//Console::WriteLine("DIRECTION = " + Convert::ToString(CTRL_IMU.DIRECTION)); //+ " ____ CmdYout = " + Convert::ToString(CJACO.CmdXout));
			printf("CMDX  = %f | CMDY = %f | CMDZ = %f \n", CTRL_IMU.Tristan_CmdX_new, CTRL_IMU.Tristan_CmdY_new, CTRL_IMU.Tristan_CmdZ_new);
			if (CTRL_IMU.ZActive)
				printf("Yaw Control Active\n");
			else
				printf("Yaw Control Not Active\n");

			printf("LIMIT MISSING = %d\n\n",(int)CTRL_IMU.limit_missing);


			printf("Left/Right JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdX_new);
			printf("Up/Down JACO Translation Value: %f\n", CTRL_IMU.Tristan_CmdY_new);
#ifdef _WIN32
			if (PRINT_CALIB)
			{
				printf("***************************************\n");
				Console::WriteLine("THETA     = " + Convert::ToString(CTRL_IMU.Theta) + "|  AMP = " + Convert::ToString(CTRL_IMU.Amp));
				Console::WriteLine("MODE ALGO = " + Convert::ToString(CTRL_IMU.MODE_JACO) + "|  MODE ROBOT = " + Convert::ToString(CTRL_IMU.MODE_JACO_RETURNED));
				printf("_______________________________________\n");
				Console::WriteLine("MIN FORW  = " + Convert::ToString(CTRL_IMU.AminForw) + "|  MIN BACK  = " + Convert::ToString(CTRL_IMU.AminBack));
				Console::WriteLine("MAX FORW  = " + Convert::ToString(CTRL_IMU.AmaxForw) + "|  MAX BACK  = " + Convert::ToString(CTRL_IMU.AmaxBack));
				Console::WriteLine("YAW FORW  = " + Convert::ToString(CTRL_IMU.YawCtrlForw) + "|  YAW BACK  = " + Convert::ToString(CTRL_IMU.YawCtrlBack));
				printf("_______________________________________\n");
			331.217560	Console::WriteLine("MIN RIGHT = " + Convert::ToString(CTRL_IMU.AminRight) + "|  MIN LEFT  = " + Convert::ToString(CTRL_IMU.AminLeft));
				Console::WriteLine("MAX RIGHT = " + Convert::ToString(CTRL_IMU.AmaxRight) + "|  MAX LEFT  = " + Convert::ToString(CTRL_IMU.AmaxLeft));
				Console::WriteLine("YAW RIGHT  = " + Convert::ToString(CTRL_IMU.YawCtrlRight) + "|  YAW LEFT  = " + Convert::ToString(CTRL_IMU.YawCtrlLeft));
				printf("_______________________________________\n");
				Console::WriteLine("ROT MIN RIGHT = " + Convert::ToString(CTRL_IMU.RotminRight) + "|  ROT MIN LEFT  = " + Convert::ToString(CTRL_IMU.RotminLeft));
				Console::WriteLine("ROT MAX RIGHT = " + Convert::ToString(CTRL_IMU.RotmaxRight) + "|  ROT MAX LEFT  = " + Convert::ToString(CTRL_IMU.RotmaxLeft));
				Console::WriteLine("AMP ROT RIGHT = " + Convert::ToString(CTRL_IMU.AmpRotRight) + "|  AMP ROT LEFT   = " + Convert::ToString(CTRL_IMU.AmpRotLeft));
				printf("_______________________________________\n");
				Console::WriteLine("ZONE FORW  = " + Convert::ToString(CTRL_IMU.ZoneForw) + "|  ZONE BACK  = " + Convert::ToString(CTRL_IMU.ZoneBack));
				Console::WriteLine("ZONE RIGHT = " + Convert::ToString(CTRL_IMU.ZoneRight) + "|  ZONE LEFT  = " + Convert::ToString(CTRL_IMU.ZoneLeft));
			}
#endif
		}
		refresh = 0;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
void update_plot(void)
{
	if (PLOT_DATA)
	{
		if (PCKT[0] == HEADSET)
		{
			new_imu[0] = CTRL_IMU.HEAD_IMU.pitchctrl;
			new_imu[1] = CTRL_IMU.HEAD_IMU.rollctrl;
			new_imu[2] = CTRL_IMU.HEAD_IMU.yawctrl;
			PLOT_IMU.update(new_imu, 3, 1);
		}
		else if (PCKT[0] == REFERENCE)
		{
		}
		else if (PCKT[0] == EMG1)
		{
			if (CTRL_IMU.EMG_1.NEW_CALIB)
			{
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
			}
			for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
			{
				new_emg[i_nemg + pemg] = CTRL_IMU.EMG_1.DATA_EMG_FILT[pemg] * 5;
				if (new_emg[i_nemg + pemg] >= 100)
					new_emg[i_nemg + pemg] = 100;
				else if (new_emg[i_nemg + pemg] <= -100)
					new_emg[i_nemg + pemg] = -100;
			}
			i_nemg += NB_SAMPLES_BY_PCKT;

			new_tke[i_ntke] = CTRL_IMU.EMG_1.MEAN_TKE_CURRENT;
			i_ntke += 1;

			if (i_nemg >= PLOT_EMG.nb_data_to_plot)
			{
				PLOT_EMG.update(new_emg, PLOT_EMG.nb_data_to_plot, 1);
				i_nemg = 0;
			}
			if (i_ntke >= PLOT_TKE.nb_data_to_plot)
			{
				PLOT_TKE.update(new_tke, PLOT_TKE.nb_data_to_plot, 1);
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				i_ntke = 0;
			}
		}
	}
}
/////331.217560/////////////////////////////////////////////////////////////////////////////////////


/* keyboard */
void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}
void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}
int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

int check_keypad(void)
{
	int num_kyp = 0;
	if (kyp_delay != 0)
	{
		kyp_delay++;
		if (kyp_delay == 5)
			kyp_delay = 0;
	}
	
	if (c == SPACEBAR_KY && (keypressed[0] == 0) && !kyp_delay)
	{
		keypressed[0] = 1;
		num_kyp = 1;
		kyp_delay = 1;
		printf("SPACE BAR PRESSED\n");
		usleep(2000000);
		c=0;
	}
	else {keypressed[0] = 0;}
	
	return num_kyp;
}

/* go331.217560to arm */

int goto_home(JacoArm *arm)
{
  // going to HOME position is possible from all positions. Only problem is,
  // if there is some kinfo of error
  jaco_retract_mode_t mode = arm->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      // is currently on the way to HOME. Need 2 button presses,
      // 1st moves towards RETRACT, 2nd brings it back to its way to HOME
      arm->push_joystick_button(2);
      arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // just 1 button press needed
      arm->push_joystick_button(2);
      break;

    case MODE_ERROR:
      printf("some error?! \n");
      return 0;
      break;

    case MODE_READY_STANDBY:
      printf("nothing to do here \n");
      return 1;
      break;
  }

  while( mode != MODE_READY_STANDBY ) {
    usleep(1000*10); // 10 ms
    mode = arm->get_status();
    if( mode == MODE_READY_TO_RETRACT ) {
      arm->release_joystick();
      arm->push_joystick_button(2);
    }
  }
  arm->release_joystick();

  return 1;
}

void test_libkindrv ()
{
	if (JACO_STARTED == 0)
	{
		KinDrv::init_usb();
		printf("Create a JacoArm \n");
		JacoArm *arm_test;
		try {
			arm_test = new JacoArm();
			printf("Successfully connected to JACO arm! \n");
		} catch( KinDrvException &e ) {
			printf("error %i: %s \n", e.error(), e.what());
			//return 0;
		}

		printf("Gaining API control over the arm \n");
		arm_test->start_api_ctrl();
		//jaco_retract_mode_t mode = arm->get_status();
		//goto_home(arm);
		arm_test->set_control_cart();

		printf("* translate forth \n");
	    MOVE_JACO_FORWARD;

		printf("* translate right \n");
		MOVE_JACO_RIGHT;

		printf("* translate back \n");
		MOVE_JACO_BACKWARD;

		printf("* translate left \n");
		MOVE_JACO_LEFT;

		printf("* translate UP \n");
		MOVE_JACO_UP;

		printf("* translate DOWN \n");
		MOVE_JACO_DOWN;

		JACO_STARTED = 1;
		//while (1);
	}
}

void init_API ()
{
	KinDrv::init_usb();
	printf("Create a JacoArm \n");
	try {
		arm = new JacoArm();
		printf("Successfully connected to JACO arm! \n");
	} catch( KinDrvException &e ) {
		printf("error %i: %s \n", e.error(), e.what());
		//return 0;
	}
	
	printf("Gaining API control over the arm \n");
	arm->start_api_ctrl();
	//jaco_retract_mode_t mode = arm->get_status();
	//goto_home(arm);
	arm->set_control_cart();
}

void init_joystick_cmd (jaco_joystick_t j)
{
	volatile unsigned int i;
	/// jaco_joystick_button_t button; /**< Simulated buttons. */
	/// \brief Simulation of joystick buttons (1 for pressed, 0 for released). Make sure to initialze with 0s!
	/// typedef unsigned short jaco_joystick_button_t[16];
	for (i = 0; i <16; i++)
		j.button [i] = 0;

	/// float axis[6]
	/// float trans_lr;    /**< (Translation Mode) Move stick +left -right. Left/Right translation of arm. */
    /// float trans_fb;    /**< (Translation Mode) Move stick +back -forth. Back/Forth translation to arm. */
    /// float trans_rot;   /**< (Translation Mode) Rotate stick +cw -ccw. Up/Down translation of arm. */
    /// float wrist_fb;    /**< (Wrist Mode) Move stick +forth -back. Up/Down inclination of wrist. */
    /// float wrist_lr;    /**< (Wrist Mode) Move stick +right -left. Forth/Back inclination of wrist. */
    /// float wrist_rot;   /**< (Wrist Mode) Rotate stick +cw -ccw. Ccw/cw rotation around wrist. */
	/// \brief Struct for joystick axis movement. Make sure to initialze with 0s!
	// jaco_joystick_axis_t   axis;   /**< Simulated axes. */
}
