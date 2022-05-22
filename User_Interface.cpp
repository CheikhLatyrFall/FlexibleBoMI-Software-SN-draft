#include "User_Interface.h"

namespace Control_IMU_JACO
{
	User_Interface::User_Interface()
	{}

	User_Interface::~User_Interface()
	{}
	
	void User_Interface::initialize()
	{
		//JACO instances
		JACO_STARTED = OFF;
		JACO_CONNECTED = OFF;
		JACO_CONTROL_ENABLED = OFF;
		
		FINGER_CONTROL = OFF;
		HEAD_MOTION_CONTROL = OFF;
		CONTROL_TRIGGER = SINGLE_CLICK;//HOLD_DOWN;//
		SAVE_DATA = OFF; //ON;//
		READ_CARTESIAN_POSITION = ON;//OFF;//
		read_cart_pos_temp = 0;
		TCP_SERVER_SET = OFF;//ON;//
		RASPBIAN_CLIENT_SET = OFF;
		
		//DEFAULT MAPPING MODE
		CONTROL_MAPPING_MODE = PITCHROLL;

		//calissifier
		SAVING_MOTION_DATA= OFF;//ON;// //FOR MONITORING AND ALGORITHM DESIGN. ONLY THE HEADSET IMU SENSOR SHOULD BE USED
		MOTION_CLASS = 0;
		NB_MOTION_CLASS = 13;//20;//
		motion_data_index = 0;
		motion_data_to_read = 600; // (1/62Hz)*500 ~ 8s

		SAVING_RANDOM_MOTION = OFF;//ON;//
		RANDOM_MOTION = 0;
		NB_RANDOM_MOTION = 29;
		random_data_to_read = 300;
		/////////////////////////////////////
		
		//USB
		PORT_COM_OUVERT = 0;
		/*S_Of_F = 51;
		E_Of_F = 52;*/
		Acq_Started = 0; //ON ou OFF
		//////////////////////////////////////

		//NETWORK, SAFETY CONTROL - KEYPAD
		EVENT_SAFETY_KEY = NO_EVENT;
		SAFETY_KEY_NETWORK = OFF;				//Indicates if the safetykey node is used. If ON, they safety relies on safetykey
		SAFETY_CONTROL_KEYPAD_TOUCH = ON;		//Indicates if the keypad or switch USB is used for safety control
		SAFETY_KEYPAD_TOUCH_PRESSED = OFF;		//Set when the safety keypad touch is pressed
		LONG_CONTRACTION_CTRL_ENABLE = OFF;
		//////////////////////////////////////

		//TIMING
		refresh = 0;
		elapsed = 20;                           //Indicates the time between screen refreshes (should be changed by using a timer)
		start_time = 0;
		stop_time  = 0;
		time_print_console = 0;
		TIMER_STARTED = 0;
		//////////////////////////////////////

		//KEYPAD
		kyp_ready = 1;
		kyp_temp = 1;
		kyp_delay = 0;

		c = 0; //Raspbian keypress value
		m = 0; //Raspbian mousepress value
		flag = 0; //Raspbian
		SDL_Event kypevent;
		quit = 0;
		//////////////////////////////////////

		//PRINT and PLOT
		PLOT_DATA = OFF;//PLOT_CTRL;//PLOT_MULTI_EMG;//
		PRINT = ON;
		PRINT_CALIB = OFF;
		//////////////////////////////////////

		//ALGO AND DATA
		EMG_PRECISION_MODE = EMG_16BIT_PRECISION;
		ControlJACO = 0; //ON ou OFF
		Bouton_Hold = 0; //ON ou OFF
		test_timer_jaco = 0;
		api_rpi_set = OFF;
		//////////////////////////////////////

		// SAVING FILE
		file_time_stamp = 0;
		file_time_stamp_start = 0;
		//////////////////////////////////////

		// TCP Client
		var_display = 0;
		//////////////////////////////////////
		
		// JACO
		jojo_jaco = {0};
		position_jaco = {0};
	}
	
	void User_Interface::routine()
	{
		data[0] = 0;
		while(data[0] != S_Of_F)
			serial1.Receive(data, 1);
		refresh++;

		serial1.Receive(data+1, Packet_Size - 1);
		if (data[Packet_Size - 1] == E_Of_F)
		{
			for (int kl = 0; kl < Packet_Size - 1; kl++)
			{
				PCKT[kl] = (int)data[kl+1];
			}
			if (RASPBIAN_CLIENT_SET)
				send_tcp_data_raspbian(data,DEFAULT_BUFLEN);
		}
		
		////////////PROCESSING TIME/////////////////////////////////
		if (TIMER_STARTED)
			stop_time = clock() - start_time;
		start_time = clock();
		TIMER_STARTED = 1;
		////////////////////////////////////////////////////////////

		if (PCKT[Packet_Size - 2] == E_Of_F)
		{	
			if (PCKT[0] == SAFETY_KEY_ID)
				EVENT_SAFETY_KEY = PCKT[13];

			CTRL_IMU.process_payload(PCKT, CONTROL_MAPPING_MODE);
			
			int res_update = CTRL_IMU.update_calibration(); //renvoie 1 à la fin d'une calibration
			if (res_update)
				PRINT = 1;

			//plot_setup.update_plot(PCKT);
			
			update_console();

			if (SAVE_DATA)
				save_data_to_file();

			if (SAVING_MOTION_DATA)
				save_motion_class_data();
			else if (SAVING_RANDOM_MOTION)
				generate_random_motion();

			change_mode_emg(CTRL_IMU.EMG_1.EMG_DETECTED);
			int etat = send_cmd_JACO();
		}

		SDL_PollEvent( &kypevent );
		if (kypevent.type == SDL_KEYDOWN)
		{
			c = kypevent.key.keysym.sym;
		}
		else if (kypevent.type == SDL_KEYUP)
		{
			c = 0;
		}
		else if (kypevent.type == SDL_MOUSEBUTTONDOWN)
		{
			c = MOUSE_KY;
			m = 1;
		}
		else if (kypevent.type == SDL_MOUSEBUTTONUP)
		{
			c = 0;
			m = 0;
		}
		int touch = check_keypad ();
	}
	
	void User_Interface::set()
	{
		int cpt = 0; int int_kyp = 0;
		for (int i_k = 0; i_k < nb_keys; i_k++)
			keypressed[i_k] = 0;

		printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
		printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
		printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
		printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
		printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
		printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

		printf("___________________________________________________________________________\n\n"); 

		CTRL_IMU.initialize(EMG_PRECISION_MODE);
		CTRL_IMU.WBSN_MODE = WBSN_CONTROL;
		
		PLOT_DATA = OFF;
		JACO_CONTROL_ENABLED = ON;
		
		// what kind of control is performed
		if (JACO_CONTROL_ENABLED)
		{
			//head motion and finger control
			//printf("Select motion control:\n\t[1] : Head Motion Control \n\t[2] : Finger Control\n");
			ch_kyp = '1';
			/*while ((ch_kyp != '1') && (ch_kyp != '2'))
			{
				std::cin >> ch_kyp;
				if ((ch_kyp != '1') && (ch_kyp != '2'))
				{
					printf("INVALID ENTRY, PLEASE RETRY\n");
					ch_kyp = 'A';
				}
			}*/
			if (ch_kyp == '1')
				HEAD_MOTION_CONTROL = ON;
			else if (ch_kyp == '2')
				FINGER_CONTROL = ON;
		}
		
		const char* usb_dev;				//Raspbian
		std::string usb_name;				//Raspbian
		//printf("Available Ports:\n");
		//result = system("ls -l /dev/ttyUSB*");
		if (result != 0){
			std::cout << "\n no USB devices were found\n" << std::endl;
			exit(0);
		}
		else 
		{
			//printf("Enter COM port [/dev/ttyUSB*]: \n");
			//std::cin >> usb_name;
			//usb_dev = usb_name.c_str();
			usb_dev = "/dev/ttyUSB0";
		}
		//mySerial serial1(usb_dev, 1998848);//115200;//);
		serial1.Open(usb_dev, 1998848);
		if (serial1.IsOpen())
		{
			printf("COM Port successfully openned\n");
			PORT_COM_OUVERT = 1;
		}
		else
		{
			printf("USB dongle not found, application quitting...\n");
			usleep (500000);
			exit(0);
		}
		// Init SDL /////////////////////////////////////////////////////////////////////
		/* Initialise SDL */
		if (SDL_Init(SDL_INIT_VIDEO) < 0)
		{
			fprintf(stderr, "Could not initialise SDL: %s\n", SDL_GetError());
			exit(-1);
		}

		/* Set a video mode */
		SDL_Surface* myVideoSurface = SDL_SetVideoMode(320, 200, 0, 0);//SDL_FULLSCREEN);
		if (!myVideoSurface)
		{
			fprintf(stderr, "Could not set video mode: %s\n", SDL_GetError());
			SDL_Quit();
			exit(-1);
		}	
		/////////////////////////////////////////////////////////////////////////////////

		plot_setup.init_plot_variables(EMG_PRECISION_MODE, &CTRL_IMU, PLOT_DATA);
		plot_setup.set_charts(PLOT_DATA);

		serial1.NumberByteRcv(*pnbyrc);
		flush_array = new unsigned char[BYTES_AVAILABLE];
		serial1.Receive(flush_array, BYTES_AVAILABLE);
		//system("clear");

		printf("Controller initailized, make sure sensors are turned ON\n");
		Acq_Started = ON;
		time_print_console = clock(); //start timer for print	
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////
	void User_Interface::generate_random_motion()
	{
		if (motion_data_index == 0)
			printf("PLEASE KEEP THIS POSITION : ");
		float var_temp = CTRL_IMU.RANDOM_ETIQUETTE[RANDOM_MOTION];
		if (var_temp == ZERO && motion_data_index == 0)
			printf("NEUTRAL\n");
		if (var_temp == FORWARD && motion_data_index == 0)
			printf("FORWARD\n");
		else if (var_temp == BACKWARD && motion_data_index == 0)
			printf("BACKWARD\n");
		else if (var_temp == RIGHT && motion_data_index == 0)
			printf("RIGHT\n");
		else if (var_temp == LEFT && motion_data_index == 0)
			printf("LEFT\n");
		else if (var_temp == UP && motion_data_index == 0)
			printf("UP\n");
		else if (var_temp == DOWN && motion_data_index == 0)
			printf("DOWN\n");
		motion_data_index++;
		if (motion_data_index == random_data_to_read)
		{
			RANDOM_MOTION++;
			motion_data_index = 0;
			if (RANDOM_MOTION == NB_RANDOM_MOTION)
			{
				RANDOM_MOTION = 0;
				SAVING_RANDOM_MOTION = OFF;
				PRINT = ON;
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////////
	void User_Interface::save_motion_class_data()
	{
		if (motion_data_index == 0)
			printf("PLEASE KEEP THIS POSITION : ");
		float var_temp = CTRL_IMU.DIRECTION_ETIQUETTE[MOTION_CLASS];
		if (var_temp == ZERO && motion_data_index == 0)
			printf("NEUTRAL\n");
		if (var_temp == FORWARD && motion_data_index == 0)
			printf("FORWARD\n");
		else if (var_temp == BACKWARD && motion_data_index == 0)
			printf("BACKWARD\n");
		else if (var_temp == RIGHT && motion_data_index == 0)
			printf("RIGHT\n");
		else if (var_temp == LEFT && motion_data_index == 0)
			printf("LEFT\n");
		else if (var_temp == UP && motion_data_index == 0)
			printf("UP\n");
		else if (var_temp == DOWN && motion_data_index == 0)
			printf("DOWN\n");

		motion_data_index++;
		if (motion_data_index == motion_data_to_read)
		{
			MOTION_CLASS++;
			motion_data_index = 0;
			if (MOTION_CLASS == NB_MOTION_CLASS)
			{
				MOTION_CLASS = 0;
				SAVING_MOTION_DATA = OFF;
				PRINT = ON;
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////

	void User_Interface::save_data_to_file()
	{
		//output_file << "Time,JacoX,JacoY,JacoZ,PitchRaw,RollRaw,YawRaw,PitchCtrl,RollCtrl,YawCtrl,CmdX,CmdY,CmdZ,Etiquette,Motion,Calibration\n";
		file_time_stamp = clock() - file_time_stamp_start;
		if (CTRL_IMU.WBSN_MODE == WBSN_CONTROL)
		{
			output_file << file_time_stamp << "," << position_jaco.position[0] << "," << position_jaco.position[1] << "," << position_jaco.position[2]<< ",";
			output_file << CTRL_IMU.HEAD_IMU.pitchraw << "," << CTRL_IMU.HEAD_IMU.rollraw << "," << CTRL_IMU.HEAD_IMU.yawraw << ",";
			output_file << CTRL_IMU.HEAD_IMU.pitchctrl << "," << CTRL_IMU.HEAD_IMU.rollctrl << "," << CTRL_IMU.HEAD_IMU.yawctrl << ",";
			output_file << CTRL_IMU.Amp << "," << CTRL_IMU.Theta << "," << CTRL_IMU.Amp_2 << "," << CTRL_IMU.Theta_2 << ",";
			output_file << CTRL_IMU.CmdX_new << "," << CTRL_IMU.CmdY_new << "," << CTRL_IMU.CmdZ_new << ",";
			if (SAVING_RANDOM_MOTION)
				output_file << CTRL_IMU.RANDOM_ETIQUETTE[MOTION_CLASS] << "," << CTRL_IMU.DIRECTION << ",";
			else
				output_file << CTRL_IMU.DIRECTION_ETIQUETTE[MOTION_CLASS] << "," << CTRL_IMU.DIRECTION << ",";
			output_file << CTRL_IMU.HEAD_IMU.acc[0] << "," << CTRL_IMU.HEAD_IMU.acc[1] << "," << CTRL_IMU.HEAD_IMU.acc[2] << ",";
			output_file << CTRL_IMU.HEAD_IMU.gyr[0] << "," << CTRL_IMU.HEAD_IMU.gyr[1] << "," << CTRL_IMU.HEAD_IMU.gyr[2] << ",";
			output_file << CTRL_IMU.HEAD_IMU.mag[0] << "," << CTRL_IMU.HEAD_IMU.mag[1] << "," << CTRL_IMU.HEAD_IMU.mag[2] << "\n";
		}
	}

	void User_Interface::change_mode_emg(char value)
	{
		if (value == 1)
		{
			if (ControlJACO == 1 && CTRL_IMU.EMG1_CALIBRATED)
			{
				CTRL_IMU.MODE_JACO += 1;
				if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
					CTRL_IMU.MODE_JACO = 0;
					
				CTRL_IMU.B_INDEX1 = XYZ_BUTTON1;
				CTRL_IMU.B_INDEX2 = XYZ_BUTTON2;

				if (EMG_CONTRACTION_IS_SHORT)
				{
					arm->push_joystick_button(CTRL_IMU.B_INDEX1);
					usleep(50000);
					arm->release_joystick();
				}
				else if (EMG_CONTRACTION_IS_LONG)
				{
					arm->push_joystick_button(CTRL_IMU.B_INDEX2);
					usleep(50000);
					arm->release_joystick();
				}
				else if (EMG_CONTRACTION_IS_VERY_LONG && LONG_CONTRACTION_CTRL_ENABLE)
				{
					if (ControlJACO == OFF)
						SET_JACO_ARM();

					if (Bouton_Hold == 1 && ControlJACO == ON)
						Bouton_Hold = 0;
					else if (Bouton_Hold == 0 && ControlJACO == ON)
					{
						Bouton_Hold = 1;
						CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
						if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
						{
							PRINT = 0;
							printf("NEUTRAL IMU CALIBRATION...\n");
							CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
						}
						CTRL_IMU.InhibiteZctrl = 0;
					}
					printf("BOUTON HOLD = %d\n", (int)Bouton_Hold);
				}
				RESET_CONTRACT_CHRONO;
			}
		}
	}


	void User_Interface::update_console(void)
	{
		if ((clock() - time_print_console >= DELAY_PRINT_CONSOLE) && PRINT)
		{
			system("clear");

			printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
			printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
			printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
			printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
			printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
			printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

			printf("_____________________________________________________________________________\n");

			printf("Processing interval : %d ms\n", stop_time/1000);
			//printf("clocks/sec = %d\n",CLOCKS_PER_SEC);
			printf("Sensor id : %d\n",PCKT[0]);
		
			if (kypevent.type == SDL_KEYUP)
			{
				printf("Key released");
				printf(", Name: %s", SDL_GetKeyName(kypevent.key.keysym.sym));
				printf(", Scancode = %d", kypevent.key.keysym.scancode);
				printf(", Code = %d \n", kypevent.key.keysym.sym);
			}
			else if (kypevent.type == SDL_KEYDOWN)
			{
				printf("Key pressed");
				printf(", Name: %s", SDL_GetKeyName(kypevent.key.keysym.sym));
				printf(", Scancode = %d", kypevent.key.keysym.scancode);
				printf(", Code = %d \n", kypevent.key.keysym.sym);
			}
			else if (kypevent.type == SDL_MOUSEBUTTONDOWN)
			{
				printf("Mouse pressed");
				printf(", Name: %s", SDL_GetKeyName(kypevent.key.keysym.sym));
				printf(", Scancode = %d", kypevent.key.keysym.scancode);
				printf(", Code = %d \n", kypevent.key.keysym.sym);
			}
			else if (kypevent.type == SDL_MOUSEBUTTONUP)
			{
				printf("Mouse released");
				printf(", Name: %s", SDL_GetKeyName(kypevent.key.keysym.sym));
				printf(", Scancode = %d", kypevent.key.keysym.scancode);
				printf(", Code = %d \n", kypevent.key.keysym.sym);
			}
			
			printf("_____________________________________________________________________________\n");
				
			if (ControlJACO == 1)
				printf("API SET\n");
			else
				printf("API CLOSED\n");

			if (Bouton_Hold == 1)
				printf("CONTROL SET\n");
			else
				printf("CONTROL STOPPED\n");

			if (SAVE_DATA)
				printf("DATA BEING SAVED\n");
			else
				printf("DATA NOT BEING SAVED YET [S]\n");
			
			if (READ_CARTESIAN_POSITION)
			{
				printf("***************************************\n");
				printf("JACO's position : \n");
				printf("CARTESIAN : X = %f | Y = %f | Z = %f\n", position_jaco.position[0], position_jaco.position[1], position_jaco.position[2]);
				printf("  ANGULAR : ThetaX = %f | ThetaY = %f | ThetaZ = %f\n", position_jaco.rotation[0], position_jaco.rotation[1], position_jaco.rotation[2]);
				printf("***************************************\n");
			}
			
			if (CTRL_IMU.HEAD_IMU_STATE <= CTRL_IMU.limit_missing)
			{
				printf("IMU PACKET NUMBER = %d \n", CTRL_IMU.HEAD_IMU.PACKET_NUMBER);
				printf("IMU LOST PACKETS  = %d \n", CTRL_IMU.HEAD_IMU.LOST_PACKETS);
				printf("FREQ HEADSET = %d Hz\n", CTRL_IMU.HEAD_IMU.FREQ);
				printf("RAW  : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchraw, (int)CTRL_IMU.HEAD_IMU.rollraw, (int)CTRL_IMU.HEAD_IMU.yawraw);
				printf("CTRL : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchctrl, (int)CTRL_IMU.HEAD_IMU.rollctrl, (int)CTRL_IMU.HEAD_IMU.yawctrl);
				printf("     : AMP   = %f __ THETA = %f | [deg]\n", (float)CTRL_IMU.Amp, (float)CTRL_IMU.Theta);
				//printf("ACCX = %f   | ACCY = %f   | ACCZ = %f \n", (float)CTRL_IMU.HEAD_IMU.acc[0], (float)CTRL_IMU.HEAD_IMU.acc[1], (float)CTRL_IMU.HEAD_IMU.acc[2]);
				//printf("GYROX = %f   | GYROY = %f   | GYROZ = %f \n", (float)CTRL_IMU.HEAD_IMU.gyr[0], (float)CTRL_IMU.HEAD_IMU.gyr[1], (float)CTRL_IMU.HEAD_IMU.gyr[2]);
				//printf("ACC CALIB FLAT = %f   |  MARGIN ACC CALIB FLAT = %f\n", (float)CTRL_IMU.HEAD_IMU.mean_acc_calib, (float)CTRL_IMU.HEAD_IMU.margin_acc_calib);
				//printf("CURRENT ACC FLAT = %f\n", (float)CTRL_IMU.HEAD_IMU.mean_acc_flat_yaw);
				if (CTRL_IMU.IMU_CALIBRATED)
				{
					printf("IMU1 CALIBRATED\n");
					plot_setup.polar_calib_set = 0;
				}
				else
					printf("IMU1 NOT CALIBRATED\n");
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
				printf("EMG CH1 PACKET NUMBER = %d \n", CTRL_IMU.EMG_1.PACKET_NUMBER);
				printf("EMG CH1 LOST PACKETS  = %d \n", CTRL_IMU.EMG_1.LOST_PACKETS);
	#ifdef _USE_DISLIN
				printf("FREQ EMG CH1  = %d Hz\n", CTRL_IMU.EMG_1.FREQ * plot_setup.NB_SAMPLES_BY_PCKT);
	#endif
				printf("CURRENT TKE CH1  = %f  |  THRESHOLD TKE CH1 = %f\n", CTRL_IMU.EMG_1.MEAN_TKE_CURRENT, CTRL_IMU.EMG_1.THRESHOLD);
				printf("UPPER THRES CH1 = %f  |  LOWER THRES CH1   = %f\n", CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD);
				printf("DETECTION EMG CH1 = %d |  CONTRACTION DURATION CH1 = %d\n ", CTRL_IMU.EMG_1.EMG_DETECTED, CTRL_IMU.EMG_1.time_contraction);
				if (CTRL_IMU.EMG1_CALIBRATED)
					printf("EMG CH1 CALIBRATED\n");
				else
					printf("EMG CH1 NOT CALIBRATED\n");
			}
			else
				printf("EMG SENSOR CH1 NOT DETECTED\n");
			printf("***************************************\n");
				
			printf("CMDX  = %f | CMDY = %f | CMDZ = %f \n", CTRL_IMU.CmdX_new, CTRL_IMU.CmdY_new, CTRL_IMU.CmdZ_new);
				
			if (CTRL_IMU.ZActive)
				printf("Yaw Control Active\n");
			else
				printf("Yaw Control Not Active\n");

			if (PRINT_CALIB)
			{
				printf("***************************************\n");
				printf("THETA      = %f\t|  AMP         =  %f\t\n", (float) CTRL_IMU.Theta, (float) CTRL_IMU.Amp); 
				printf("MODE ALGO  = %d\t|  MODE ROBOT  =  %f\t\n", (int)CTRL_IMU.MODE_JACO, (int)CTRL_IMU.MODE_JACO_RETURNED);
				printf("_______________________________________\n");

				printf("MIN FORW   = %f\t|  MIN BACK    =  %f\t\n", (float)CTRL_IMU.AminForw, (float)CTRL_IMU.AminBack);
				printf("MAX FORW   = %f\t|  MAX BACK    =  %f\t\n", (float)CTRL_IMU.AmaxForw, (float)CTRL_IMU.AmaxBack);
				printf("YAW FORW   = %f\t|  YAW BACK    =  %f\t\n", (float)CTRL_IMU.YawCtrlForw, (float)CTRL_IMU.YawCtrlBack);
				printf("_______________________________________\n");

				printf("MIN RIGHT   = %f\t|  MIN LEFT    =  %f\t\n", (float)CTRL_IMU.AminRight, (float)CTRL_IMU.AminLeft);
				printf("MAX RIGHT   = %f\t|  MAX LEFT    =  %f\t\n", (float)CTRL_IMU.AmaxRight, (float)CTRL_IMU.AmaxLeft);
				printf("YAW RIGHT   = %f\t|  YAW LEFT    =  %f\t\n", (float)CTRL_IMU.YawCtrlRight, (float)CTRL_IMU.YawCtrlLeft);
				printf("_______________________________________\n");

				printf("ROT MIN RIGHT   = %f\t|  ROT MIN LEFT    =  %f\t\n", (float)CTRL_IMU.RotminRight, (float)CTRL_IMU.RotminLeft);
				printf("ROT MAX RIGHT   = %f\t|  ROT MAX LEFT    =  %f\t\n", (float)CTRL_IMU.RotmaxRight, (float)CTRL_IMU.RotmaxLeft);
				printf("AMP ROT RIGHT   = %f\t|  AMP ROT LEFT    =  %f\t\n", (float)CTRL_IMU.AmpRotRight, (float)CTRL_IMU.AmpRotLeft);
				printf("_______________________________________\n"); 

				printf("ZONE FORW   = %f\t|  ZONE BACK    =  %f\t\n", (float)CTRL_IMU.ZoneForw, (float)CTRL_IMU.ZoneBack);
				printf("ZONE RIGHT  = %f\t|  ZONE LEFT    =  %f\t\n", (float)CTRL_IMU.ZoneRight, (float)CTRL_IMU.ZoneLeft);
			}
			time_print_console = clock();
		}
	}


	int User_Interface::send_cmd_JACO (void)
	{
		int result;
		if (ControlJACO == 1 && READ_CARTESIAN_POSITION)
		{
			read_cart_pos_temp ++;
			if (read_cart_pos_temp == 20)
			{
					read_cart_pos_temp = 0;
					position_jaco = arm->get_cart_pos();			
			}
		}
		if (Bouton_Hold == 1 && ControlJACO == 1 && CTRL_IMU.IMU_CALIBRATED)
		{
			jojo_jaco = CTRL_IMU.CommandJACO_RPI(1);
			arm->move_joystick_axis(jojo_jaco.axis);
		}
		else if (Bouton_Hold == 0 && ControlJACO == 1)
		{
			jojo_jaco = CTRL_IMU.CommandJACO_RPI(0);
			arm->move_joystick_axis(jojo_jaco.axis);
		}
		return result;
	}


	int User_Interface::check_keypad(void)
	{
		int num_kyp = 0;
		if (kyp_temp == 0)
		{
			kyp_delay++;
			if (kyp_delay == 30)
			{
				kyp_delay = 1;
				kyp_temp = 1;
				kyp_ready = 1;
			}
		}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(D_KY) && kyp_ready && (keypressed[0] == 0))//STOP/START API
	#else
		if (c == D_KY && kyp_ready && (keypressed[0] == 0))
	#endif
		{
			keypressed[0] = 1;
			num_kyp = 1;
		#pragma region STOP/START API
			kyp_delay = 1;
			SET_JACO_ARM();
		#pragma endregion
		}
		else {keypressed[0] = 0;}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(U_KY)) && kyp_temp == 1)//LOCK/UNLOCK KEYPAD
	#else
		if (c == U_KY && kyp_temp == 1)
	#endif
		{
			if (keypressed[1] == 0)
			{
				#pragma region LOCK/UNLOCK KEYPAD
				kyp_delay = 1;
				if (kyp_ready == 0)
				{
					kyp_ready = 1;
					printf("Keyboard Unlocked \n");
				}
				else
				{
					kyp_ready = 0;
					printf("Keyboard Locked \n");
				}
				#pragma endregion
			}
			keypressed[1] = 1;
			num_kyp = 2;
		}
		else {keypressed[1] = 0;}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if (GetAsyncKeyState(SPACEBAR_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)
	#else
		if (c == SPACEBAR_KY && kyp_ready)
	#endif
		{
			if (keypressed[2] == 0)
			{
				#pragma region EN/DISABLE - SET 0
				kyp_delay = 1;
				if (Bouton_Hold == 0)
				{
					CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
					if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
					{
						PRINT = 0;
						printf("NEUTRAL IMU CALIBRATION...\n");
						CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
					}
					CTRL_IMU.InhibiteZctrl = 0;
				}
				//printf("BOUTON HOLD = %d",(int)Bouton_Hold + "\n");
				#pragma endregion
			}
			keypressed[2] = 1;
			num_kyp = 3;
			SAFETY_KEYPAD_TOUCH_PRESSED = true;
			if (CTRL_IMU.HEAD_IMU.CALIB_TYPE  == NO_CALIB)
				Bouton_Hold = 1;
			//printf("SAFETY KEY PRESSED\n");
		}
		else
		{
			if (CONTROL_TRIGGER == SINGLE_CLICK)
				keypressed[2] = 0;
			else if (CONTROL_TRIGGER == HOLD_DOWN)
			{	
				Bouton_Hold = 0; //Comment this line to allow permanent ControlEnable
				keypressed[2] = 0;
				SAFETY_KEYPAD_TOUCH_PRESSED = false; //Comment this line to allow permanent ControlEnable
			}
		}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(O_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)//CHANGE MODE B (BOUTON 2)
	#else
		if (c == O_KY && kyp_ready && !CTRL_IMU.WBSN_MODE)
	#endif
		{
			if (keypressed[3] == 0)
			{
				#pragma region CHANGE MODE Bouton 2
				kyp_delay = 1;
				if (ControlJACO)
				{
					CTRL_IMU.MODE_JACO += 1;
					if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
						CTRL_IMU.MODE_JACO = 0;
						
					CTRL_IMU.B_INDEX1 = XYZ_BUTTON2;
					CTRL_IMU.B_INDEX2 = 0;

					arm->push_joystick_button(CTRL_IMU.B_INDEX1);
					usleep(50000);
					arm->release_joystick();
				}
				#pragma endregion
			}
			keypressed[3] = 1;
			num_kyp = 4;
		}
		else {keypressed[3] = 0;}
		/*__________________________________________________________________________________*/
		if (c == P_KY && kyp_ready) //(c == MOUSE_KY && kyp_ready)
		{
			if (keypressed[4] == 0)
			{
				#pragma region CHANGE MODE Bouton 1
				kyp_delay = 1;
				if (ControlJACO)
				{
					CTRL_IMU.MODE_JACO += 1;
					if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
						CTRL_IMU.MODE_JACO = 0;

					CTRL_IMU.B_INDEX1 = 0;
					CTRL_IMU.B_INDEX2 = XYZ_BUTTON1;

					arm->push_joystick_button(CTRL_IMU.B_INDEX2);
					usleep(50000);
					arm->release_joystick();			
				}
				#pragma endregion
			}
			keypressed[4] = 1;
			num_kyp = 5;
		}
		else {keypressed[4] = 0;}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(F_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)//CONTROL TRIGGER SINGLE CLICK
	#else
		if (c == F_KY && kyp_ready)
	#endif
		{
			if (keypressed[5] == 0)
			{
				kyp_delay = 1;
				if (CONTROL_TRIGGER == SINGLE_CLICK)
				{
					if (Bouton_Hold == OFF)
						Bouton_Hold = ON;
					else if (Bouton_Hold == ON)
						Bouton_Hold = OFF;
				}
			}

			keypressed[5] = 1;
			num_kyp = 6;
		}
		else {keypressed[5] = 0;}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(Z_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)//CALIBRATE EMG ZERO
	#else
		if (c == Z_KY && kyp_ready)
	#endif
		{
			if (keypressed[6] == 0)
			{
				#pragma region CALIB NEUTRAL EMG
				kyp_delay = 1;
				CTRL_IMU.EMG_1.CALIB_EMG_INDEX = 0;
				if (CTRL_IMU.EMG_1.CALIB_EMG_INDEX != -1)
				{
					PRINT = 0;
					printf("NEUTRAL EMG CALIBRATION...\n");
				}
				#pragma endregion
			}

			keypressed[6] = 1;
			num_kyp = 7;
		}
		else {keypressed[6] = 0;}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(add_KY) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//UP ARROW --> INCREASE EMG THRESHOLD
	#else
		if (c == add_KY && kyp_ready)
	#endif
		{
			if (keypressed[7] == 0)
			{
				kyp_delay = 1;
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					if (CTRL_IMU.EMG_1.STD_TKE == 0)
						CTRL_IMU.EMG_1.STD_TKE = 1;
					else
						CTRL_IMU.EMG_1.K_TH++;
					printf("threshold increased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				keypressed[7] = 1;
				num_kyp = 8;
			}
		}
		else { keypressed[7] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(sub_KY) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//DOWN ARROW -->	DECREASE EMG THRESHOLD
	#else
		if (c == sub_KY && kyp_ready)
	#endif
		{
			if (keypressed[8] == 0)
			{
				kyp_delay = 1;
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					if (CTRL_IMU.EMG_1.K_TH == 1 && CTRL_IMU.EMG_1.STD_TKE != 0)
					{
						CTRL_IMU.EMG_1.STD_TKE--;
						CTRL_IMU.EMG_1.NEW_CALIB = 1;
						printf("threshold decreased...");
					}
					else if (CTRL_IMU.EMG_1.K_TH != 1)
					{
						CTRL_IMU.EMG_1.K_TH--;
						CTRL_IMU.EMG_1.NEW_CALIB = 1;
						printf("threshold decreased...");
					}
					else
						printf("not valid...");
				}
				keypressed[8] = 1;
				num_kyp = 9;
			}
		}
		else { keypressed[8] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(up_arrow) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//MAX FORTH CALIBRATION
	#else
		if (c == up_arrow && kyp_ready)
	#endif
		{
			if (keypressed[9] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("MAX FORWARD CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_FORTH;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[9] = 1;
				num_kyp = 10;
			}
		}
		else { keypressed[9] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(down_arrow) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//MAX BACK CALIBRATION
	#else
		if (c == down_arrow && kyp_ready)
	#endif
		{
			if (keypressed[10] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("MAX BACKWARD CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_BACK;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[10] = 1;
				num_kyp = 11;
			}
		}
		else { keypressed[10] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(right_arrow) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//MAX RIGHT CALIBRATION
	#else
		if (c == right_arrow && kyp_ready)
	#endif
		{
			if (keypressed[11] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("MAX RIGHT CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_RIGHT;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[11] = 1;
				num_kyp = 12;
			}
		}
		else { keypressed[11] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(left_arrow) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//MAX LEFT CALIBRATION
	#else
		if (c == left_arrow && kyp_ready)
	#endif
		{
			if (keypressed[12] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("MAX LEFT CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_LEFT;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[12] = 1;
				num_kyp = 13;
			}
		}
		else { keypressed[12] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(numpad0) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//NEUTRAL IMU CALIBRATION
	#else
		if (c == numpad0 && kyp_ready)
	#endif
		{
			if (keypressed[13] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("NEUTRAL IMU CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[13] = 1;
				num_kyp = 14;
			}
		}
		else { keypressed[13] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad4) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//ROTATION LEFT CALIBRATION
	#else
		if (c == numpad4 && kyp_ready)
	#endif
		{
			if (keypressed[14] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("ROT MAX LEFT CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_LEFT;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[14] = 1;
				num_kyp = 15;
			}
		}
		else { keypressed[14] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad6) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//ROTATION RIGHT CALIBRATION
	#else
		if (c == numpad6 && kyp_ready)
	#endif
		{
			if (keypressed[15] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("ROT MAX RIGHT CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_RIGHT;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 100;//1.5s
				}
				keypressed[15] = 1;
				num_kyp = 16;
			}
		}
		else { keypressed[15] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if (GetAsyncKeyState(E_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)//Z Control disabled/enabled
	#else
		if (c == E_KY && kyp_ready)
	#endif
		{
			if (keypressed[16] == 0)
			{
				kyp_delay = 1;
				if (ControlJACO == 1)
				{
	#ifdef _WIN32
					printf("Initializing Arm...\n");
					(*MyMoveHome)();		// Puts JACO in init position
					(*MyInitFingers)();
	#else
					////////////////
					goto_home(arm);
					////////////////
	#endif
				}
				else
				{
					printf("API Closed, can't initialize arm\n");
				}
			}

			keypressed[16] = 1;
			num_kyp = 17;
		}
		else { keypressed[16] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if (GetAsyncKeyState(V_KY) && kyp_ready && !CTRL_IMU.WBSN_MODE)//Validate control heading offset
	#else
		if (c == V_KY && kyp_ready)
	#endif	
		{
			if (keypressed[17] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("HEADING OFFSET CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = HEADING_OFFSET;
					SENSOR_HEADING_COMPUTATION;
					if (CTRL_IMU.CHAIR_IMU.SENSOR_ON)
						CTRL_IMU.ReferenceActive = 1;
				}
				keypressed[17] = 1;
				num_kyp = 18;
			}
		}
		else { keypressed[17] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad8) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//LOWER_THRESHOLD INCREASE
	#else
		if (c == numpad8 && kyp_ready)
	#endif
		{
			if (keypressed[18] == 0)
			{
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					if (CTRL_IMU.EMG_1.LOWER_THRESHOLD + 1 < CTRL_IMU.EMG_1.UPPER_THRESHOLD)
					{
						CTRL_IMU.EMG_1.LOWER_THRESHOLD += 1;
						printf("threshold increased...");
						CTRL_IMU.EMG_1.NEW_CALIB = 1;
					}
					else
						printf("not valid...");
				}
				keypressed[18] = 1;
				num_kyp = 19;
			}
		}
		else { keypressed[18] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad2) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//LOWER_THRESHOLD DECREASE
	#else
		if (c == numpad2 && kyp_ready)
	#endif	
		{
			if (keypressed[19] == 0)
			{
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					if (CTRL_IMU.EMG_1.LOWER_THRESHOLD - 1 >= 0)
					{
						CTRL_IMU.EMG_1.LOWER_THRESHOLD -= 1;
						printf("threshold decreased...");
						CTRL_IMU.EMG_1.NEW_CALIB = 1;
					}
					else
						printf("not valid...");
				}
				keypressed[19] = 1;
				num_kyp = 20;
			}
		}
		else { keypressed[19] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad9) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//UPPER_THRESHOLD INCREASE
	#else
		if (c == numpad9 && kyp_ready)
	#endif	
		{
			if (keypressed[20] == 0)
			{
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					CTRL_IMU.EMG_1.UPPER_THRESHOLD += 1;
					printf("upper-threshold increased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				keypressed[20] = 1;
				num_kyp = 21;
			}
		}
		else { keypressed[20] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(numpad3) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//UPPER_THRESHOLD DECREASE
	#else
		if (c == numpad3 && kyp_ready)
	#endif	
		{
			if (keypressed[21] == 0)
			{
				if (CTRL_IMU.EMG_1.SENSOR_ON)
				{
					if (CTRL_IMU.EMG_1.UPPER_THRESHOLD - 1 > CTRL_IMU.EMG_1.LOWER_THRESHOLD)
					{
						CTRL_IMU.EMG_1.UPPER_THRESHOLD -= 1;
						printf("upper-threshold decreased...\n");
						CTRL_IMU.EMG_1.NEW_CALIB = 1;
					}
					else
						printf("not valid...");
				}
				keypressed[21] = 1;
				num_kyp = 20;
			}
		}
		else { keypressed[21] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32	
		if ((GetAsyncKeyState(W_KY) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//Toggle Z control
	#else
		if (c == W_KY && kyp_ready)
	#endif	
		{
			if (keypressed[22] == 0)
			{
				if (CTRL_IMU.ZActive)
				{
					CTRL_IMU.ZActive = 0;
					printf("YAW CONTROL DISABLED...\n");
					//SetCursorPos(200,200);
				}
				else
				{
					CTRL_IMU.ZActive = 1;
					printf("YAW CONTROL ENABLED...\n");
					//SetCursorPos(200, 200);
				}

				keypressed[22] = 1;
				num_kyp = 21;
			}
		}
		else { keypressed[22] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(numpad7) && kyp_ready) && !CTRL_IMU.WBSN_MODE)//ACC HEADING OFFSET
	#else
		if (c == numpad7 && kyp_ready)
	#endif
		{
			if (keypressed[23] == 0)
			{
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("ACC HEADING OFFSET CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = HEADING_OFFSET;
					CTRL_IMU.HEAD_IMU.NB_VALUES_CALIB_CURRENT = 500;//15s
				}
				keypressed[23] = 1;
				num_kyp = 22;
			}
		}
		else { keypressed[23] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(Q_KY))//QUIT PROGRAM
	#else
		if ((c == Q_KY) && kyp_ready)
	#endif
		{
			printf("STOPPING CONTROLLER, HOPE YOU ENJOYED\n");
	#ifdef _WIN32		
			Sleep(500);
	#else
			usleep (500000);
	#endif
			if (ControlJACO) //API set
			{
				printf("CLOSING API ...\n");
				STOP_JACO_ARM ();
				ControlJACO = 0;
			}
			keypressed[24] = 1;
			num_kyp = 23;
			exit(1);
		}
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if (GetAsyncKeyState(S_KY) && kyp_ready)//sTART SAVING DATA
	#else
		if (c == S_KY && kyp_ready)
	#endif
		{
			if (keypressed[25] == 0)
			{
				kyp_delay = 1;
				if (SAVE_DATA == OFF)
				{
					SAVE_DATA = ON;
					system("cls");
					printf("DATA SAVING STARTED\n");
					printf("Provide output file name [filename.csv] and press ENTER: ");
					std::cin >> output_file_name;
					output_file.open(output_file_name, ios::out | ios::app);
					kyp_temp = 0;
					kyp_ready = 0;
			
					current_date = time(0);
					struct_date = localtime(&current_date);
					file_time_stamp_start = clock();
					
					output_file << "Author: Cheikh Latyr Fall\n This file is generated automatically to save output data of the controller \t ";
					output_file << struct_date->tm_year+1900 << "/" << struct_date->tm_mon+1 << "/" << struct_date->tm_mday << "\n\n";
					if (!CTRL_IMU.WBSN_MODE)
						output_file << "Time,JacoX,JacoY,JacoZ,PitchRaw,RollRaw,YawRaw,PitchCtrl,RollCtrl,YawCtrl,Amp,Theta,Amp_2,Theta_2,CmdX,CmdY,CmdZ,Etiquette,Motion,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ,MAGX,MAGY,MAGZ\n";
					else if (CTRL_IMU.WBSN_MODE == WBSN_RECORDING)
					{
						output_file << "Time,EMG,";
						output_file << "Pitch1Raw,Roll1Raw,Yaw1Raw,ACC1X,ACC1Y,ACC1Z,GYRO1X,GYRO1Y,GYRO1Z,MAG1X,MAG1Y,MAG1Z,";
						output_file << "Pitch2Raw,Roll2Raw,Yaw2Raw,ACC2X,ACC2Y,ACC2Z,GYRO2X,GYRO2Y,GYRO2Z,MAG2X,MAG2Y,MAG2Z,";
						output_file << "Pitch3Raw,Roll3Raw,Yaw3Raw,ACC3X,ACC3Y,ACC3Z,GYRO3X,GYRO3Y,GYRO3Z,MAG3X,MAG3Y,MAG3Z\n";
					}
				}
				else if (SAVE_DATA == ON)
				{
					SAVE_DATA = OFF;
					output_file.close();
				}
			}

			keypressed[25] = 1;
			num_kyp = 24;
		}
		else { keypressed[25] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(A_KY) && kyp_ready))//IMU DATA SAVING FOR ALGO DESIGN
	#else
		if (c == A_KY && kyp_ready)
	#endif
		{
			if (keypressed[26] == 0)
			{
				if (!SAVE_DATA)
				{
					printf("MAKE SURE DATA SAVING IS ENABLED FIRST !\n");
					SAVING_MOTION_DATA = OFF;
				}
				else
				{
					//START MOTION SEQUENCE
					system("cls");
					printf("PLEASE MAKE SURE ALL CALIBRATIONS HAVE BEEN PERFORMED AND PRESS 'Y' OR PRESS 'N'\n");
					ch_kyp = 'A';
					while ((ch_kyp != 'Y') && (ch_kyp != 'y') && (ch_kyp != 'N') && (ch_kyp != 'n'))
						std::cin >> ch_kyp;
					if ((ch_kyp == 'Y' || ch_kyp == 'y'))
					{
						PRINT = OFF;
						SAVING_MOTION_DATA = ON;
						system("cls");
						printf("***********C  A  L  I  B  R  A  T  I  O  N************\n");
					}
				}
				keypressed[26] = 1;
				num_kyp = 25;
			}
		}
		else { keypressed[26] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(B_KY) && kyp_ready))//START TCP CLIENT TO SERVER
	#else
		if (c == B_KY && kyp_ready)
	#endif
		{
			if (keypressed[27] == 0)
			{
				if (!RASPBIAN_CLIENT_SET)
				{	
					init_tcp_client_com_raspbian();
					RASPBIAN_CLIENT_SET = ON;
				}
				else
				{
					close(sockfd);
					RASPBIAN_CLIENT_SET = OFF;
				}
				keypressed[27] = 1;
				num_kyp = 26;
			}
		}
		else { keypressed[27] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(C_KY) && kyp_ready))//CHANGE MOUSE CONTROL MAPPING
	#else
		if (c == C_KY && kyp_ready)
	#endif
		{
			if (keypressed[28] == 0)
			{
				if (CONTROL_MAPPING_MODE == PITCHROLL)
					CONTROL_MAPPING_MODE = PITCHYAW;
				else
					CONTROL_MAPPING_MODE = PITCHROLL;

				keypressed[28] = 1;
				num_kyp = 27;
			}
		}
		else { keypressed[28] = 0; }
		/*__________________________________________________________________________________*/
	#ifdef _WIN32
		if ((GetAsyncKeyState(R_KY) && kyp_ready))//IMU DATA SAVING TO RUN CLASSIFIER
	#else
		if (c == R_KY && kyp_ready)
	#endif
		{
			if (keypressed[29] == 0)
			{
				if (!SAVE_DATA)
				{
					printf("MAKE SURE DATA SAVING IS ENABLED FIRST !\n");
					SAVING_RANDOM_MOTION = OFF;
				}
				else
				{
					//START MOTION SEQUENCE
					system("cls");
					printf("PLEASE MAKE SURE ALL CALIBRATIONS HAVE BEEN PERFORMED AND PRESS 'Y' OR PRESS 'N'\n");
					ch_kyp = 'A';
					while ((ch_kyp != 'Y') && (ch_kyp != 'y') && (ch_kyp != 'N') && (ch_kyp != 'n'))
						std::cin >> ch_kyp;
					if ((ch_kyp == 'Y' || ch_kyp == 'y'))
					{
						PRINT = OFF;
						SAVING_RANDOM_MOTION = ON;
						int it;
						for (it = 0; it < 4 * NUMBER_OF_MOTIONS; it++) //NUMBER OF MOTIONS in Defined_Macro.h
						{
							if (it % 2 == 0)
								CTRL_IMU.RANDOM_ETIQUETTE[it] = ZERO;
							else
								CTRL_IMU.RANDOM_ETIQUETTE[it] = rand() % 7;
						}
						system("cls");
						printf("***********P E R F O R M     R A N D O M     M O T I O N S************\n");
					}
				}
				keypressed[29] = 1;
				num_kyp = 28;
			}
		}
		else { keypressed[29] = 0; }
		/*__________________________________________________________________________________*/
		
		return num_kyp;
	}


	int User_Interface::START_JACO_ARM()
	{
		int result;
		ControlJACO = 1;

		KinDrv::init_usb();
		printf("Create a JacoArm \n");
		try {
			arm = new JacoArm();
			printf("Successfully connected to JACO arm! \n");
		}
		catch (KinDrvException &e) {
			printf("error %i: %s \n", e.error(), e.what());
			return 0;
		}

		printf("Gaining API control over the arm \n");
		arm->start_api_ctrl();
		arm->set_control_cart();
		printf("JACO is ready... have fun ;) \n");
		////////////////////
		goto_home(arm);
		////////////////////
		return result;
	}

	int User_Interface::STOP_JACO_ARM()
	{
		ControlJACO = 0;
		//STOP API
		////////////////////
		goto_home(arm);
		////////////////////
		arm->stop_api_ctrl();
		printf("JACO's control has been stopped...\n");
		return result;
	}

	int User_Interface::SET_JACO_ARM()
	{
		if (ControlJACO == 1)
		{
			STOP_JACO_ARM();
		}
		else
		{
			START_JACO_ARM();
		}
		return ControlJACO;
	}


	////////// tcp connection ///////
	void User_Interface::error (char * msg)
	{
		perror(msg);
		exit(0);
	}

	int User_Interface::init_tcp_client_com_raspbian ()
	{
		portno = DEFAULT_PORT;
		sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sockfd < 0)
			error((char*)"ERROR opening socket");
		//printf("Please provide server's IP address\n");
		//char ip[17];
		//std::cin >> ip;
		server = gethostbyname("169.254.10.157");
		if (server == NULL)
		{
			fprintf(stderr, "ERROR, no such host\n");
			//exit(0);
		}
		else
		{
			bzero((char*) &serv_addr, sizeof(serv_addr));
			serv_addr.sin_family = AF_INET;
			bcopy((char*) server->h_addr, (char*)&serv_addr.sin_addr.s_addr,server->h_length);
			serv_addr.sin_port = htons(portno);
			if (connect(sockfd,(struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
			{	
				error((char*)"ERROR connecting");
				//printf("ERROR Connecting, linking to server aborted\n");
			}
			else
				RASPBIAN_CLIENT_SET = ON;
		}	
	}

	int User_Interface::send_tcp_data_raspbian (unsigned char * data, int data_len)
	{
		n = write(sockfd,data,data_len);
		if (n<0)
			error((char*)"ERROR writing to socket");
		else
		{
			var_display ++;
			if (var_display == 20)
			{
				system("clear");
				printf("data being sent via udp to server, size = %d\n",n);
				printf("data[0] = %d, data[31] = %d\n",data[0],data[31]);
				printf("processing interval = %d ms\n",stop_time);
				var_display = 0;
			}
		}
	}
	
	/* Print all information about a key event */
	void User_Interface::PrintKeyInfo(SDL_KeyboardEvent *key)
	{
		/* Is it a release or a press? */
		if (key->type == SDL_KEYUP)
			printf("Release:- ");
		else
			printf("Press:- ");

		/* Print the hardware scancode first */
		printf("Scancode: 0x%02X", key->keysym.scancode);
		/* Print the name of the key */
		printf(", Name: %s", SDL_GetKeyName(key->keysym.sym));
		/* We want to print the unicode info, but we need to make */
		/* sure its a press event first (remember, release events */
		/* don't have unicode info                                */
		if (key->type == SDL_KEYDOWN){
			/* If the Unicode value is less than 0x80 then the    */
			/* unicode value can be used to get a printable       */
			/* representation of the key, using (char)unicode.    */
			printf(", Unicode: ");
			if (key->keysym.unicode < 0x80 && key->keysym.unicode > 0){
				printf("%c (0x%04X)", (char)key->keysym.unicode,
					key->keysym.unicode);
			}
			else{
				printf("? (0x%04X)", key->keysym.unicode);
			}
		}
		printf("\n");
		/* Print modifier info */
		PrintModifiers(key->keysym.mod);
	}



	/* Print modifier info */
	void User_Interface::PrintModifiers(SDLMod mod)
	{
		printf("Modifers: ");

		/* If there are none then say so and return */
		if (mod == KMOD_NONE){
			printf("None\n");
			return;
		}

		/* Check for the presence of each SDLMod value */
		/* This looks messy, but there really isn't    */
		/* a clearer way.                              */
		if (mod & KMOD_NUM) printf("NUMLOCK ");
		if (mod & KMOD_CAPS) printf("CAPSLOCK ");
		if (mod & KMOD_LCTRL) printf("LCTRL ");
		if (mod & KMOD_RCTRL) printf("RCTRL ");
		if (mod & KMOD_RSHIFT) printf("RSHIFT ");
		if (mod & KMOD_LSHIFT) printf("LSHIFT ");
		if (mod & KMOD_RALT) printf("RALT ");
		if (mod & KMOD_LALT) printf("LALT ");
		if (mod & KMOD_CTRL) printf("CTRL ");
		if (mod & KMOD_SHIFT) printf("SHIFT ");
		if (mod & KMOD_ALT) printf("ALT ");
		printf("\n");
	}
	
	
	int User_Interface::goto_retract(JacoArm *arm)
	{
		// this can only be achieved from HOME position. Otherwise the arm
		// will move to HOME. You'll probably need to uncomment the gripper movements
		// in order for this to work. Or even better, implement moving to HOME position,
		// which could be triggered before going to RETRACT ;)
		jaco_retract_mode_t mode = arm->get_status();
		switch( mode ) {
		case MODE_READY_TO_RETRACT:
		  // is currently on the way to RETRACT. Need 2 button presses,
		  // 1st moves towards HOME, 2nd brings it back to its way to RETRACT
		  arm->push_joystick_button(2);
		  arm->release_joystick();
		  arm->push_joystick_button(2);
		  break;

		case MODE_READY_STANDBY:
		case MODE_RETRACT_TO_READY:
		  // just 1 button press needed
		  arm->push_joystick_button(2);
		  break;

		case MODE_NORMAL_TO_READY:
		case MODE_NORMAL:
		case MODE_NOINIT:
		  printf("cannot go from NORMAL/NOINIT to RETRACT \n");
		  return 0;
		  break;

		case MODE_ERROR:
		  printf("some error?! \n");
		  return 0;
		  break;

		case MODE_RETRACT_STANDBY:
		  printf("nothing to do here \n");
		  return 1;
		  break;
	  }

	  while( mode != MODE_RETRACT_STANDBY ) {
		usleep(1000*10); // 10 ms
		mode = arm->get_status();
	  }
	  arm->release_joystick();

	  return 1;
	}


	int User_Interface::goto_home(JacoArm *arm)
	{
		// going to HOME position is possible from all positions. Only problem is,
		// if there is some kinfo of error
		jaco_retract_mode_t mode = arm->get_status();
		switch( mode ) {
		case MODE_RETRACT_TO_READY:
		  // is currently on the way to HOME. Need 2 button presses,
		  // 1st moves towards RETRACT, 2nd brings it back to its way to HOME
		  arm->push_joystick_button(2);
		  arm->release_joystick();
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
}



