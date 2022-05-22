#include "Control_HMI.h"

namespace Control_IMU_JACO
{
	Control_HMI::Control_HMI()
	{}

	Control_HMI::~Control_HMI()
	{}

	void Control_HMI::initialize(char emg_mode)
	{
		HEAD_IMU.initialize(HEADSET);
		CHAIR_IMU.initialize(IMU2);// REFERENCE);
		IMU3_IMU.initialize(IMU3);
		EMG_1.initialize(emg_mode);
		EMG_2.initialize(emg_mode);
		EMG_3.initialize(emg_mode);
		EMG_4.initialize(emg_mode);
#ifdef _WIN32		
		limit_missing = 20;
#else
		limit_missing = 100;
#endif

		HEAD_IMU_STATE = 0;
		CHAIR_IMU_STATE = 0;
		IMU3_IMU_STATE = 0;
		EMG_1_STATE = 0;
		EMG_2_STATE = 0;
		EMG_3_STATE = 0;
		EMG_4_STATE = 0;
		BACK = 0;

		MODE_JACO_RETURNED = 0;

		//DIRECTION - COMMANDES
		MODE_JACO = 0;
		CmdX_new = 0; CmdY_new = 0;
		CmdXout = 0;	CmdYout = 0;
		CmdZ = 0; CmdZ_new = 0;
		RightLeftComm = 0.0;
		ForBackComm = 0.0;
		UpDownComm = 0.0;
		RightLeftComm_ctrl = 0.0;
		ForBackComm_ctrl = 0.0;
		UpDownComm_ctrl = 0.0;

		x = 0;									//roll + offset
		y = 0;									//pitch + offset
		x0 = 0;									//roll brute
		y0 = 0;									//pitch brute	
		xoff = 0;									//offset - position neutre roll
		yoff = 0;									//offset - position neutre pitch
		Amp = 0.0;
		Theta = 0.0;
		Orientation = 0.0;
		DIRECTION = ZERO;
		//INTERNAL USE
		DIRECTION_ETIQUETTE = new float[2 * NUMBER_OF_MOTIONS];
		RANDOM_ETIQUETTE = new float[4 * NUMBER_OF_MOTIONS];

		z_2 = 0.0;
		Amp_2 = 0.0;
		Theta_2 = 0.0;

		MODE_JACO = 0;
		MODE_JACO_RETURNED = 0;
		IMU_CALIBRATED = 0;
		EMG1_CALIBRATED = 0;
		EMG2_CALIBRATED = 0;
		EMG3_CALIBRATED = 0;
		EMG4_CALIBRATED = 0;

		z0 = 0;
		z_ctrl = 0;
		zoff = 0;
		z0_ref = 0;
		z0mean = 500;
		InhibiteZctrl = 0;

		for (int t = 0; t < 6; t++)
			imu_calibration[t] = 0;

		set_default_calibration();
	}

	void Control_HMI::set_default_calibration(void)
	{
		ZActive = 1;//0;//
		ReferenceActive = 0;						//indique l'acitivation du noeud de référence
		Valid = 1;									//indique la validité des marges et zones définies
		DiagoActive = 1;//0;//						//0 = diagonale désactivée, 1 = diagonale activée
		Theta_offset = 0;							//0 .. 360 degrés, mapping
		ThRotReference = 30;						//vitesse de rotation seuil maximale du fauteuil

		AmaxForw = 80;/*50;*/ AmaxBack = 80;/*30;*/

		AminForw = 10;/*7*/  AminBack = 7;//3;//1
		AminRight = 10;/*7*/  AminLeft = 10;/*7*/

		AmaxRight = 80;/*50;*/ AmaxLeft = 80;/*50;*/
		

		YawCtrlForw = 0;  YawCtrlBack = 0;
		YawCtrlRight = 0; YawCtrlLeft = 0;

		RotmaxRight = -10; RotmaxLeft = 10;
		RotminRight = -3; RotminLeft = 3;
		PitchRotRight = 0; PitchRotLeft = 0;
		RollRotRight = 0;  RollRotLeft = 0;
		AmpRotRight = 0; AmpRotLeft = 0;

		ZoneForw = 90;	DeltaForw = 25;// 5;	//Angle Forward	, Marge Forwad
		ZoneRight= 0;	DeltaRight= 25;// 5;	//Angle Right	, Marge Right
		ZoneLeft = 180;	DeltaLeft = 25;// 5;	//Angle Left	, Marge Left
		ZoneBack = 270;	DeltaBack = 25;// 5;	//Angle Backward, Marge Backward

		if (ZoneRight > 180)
		{
			ZoneRight = ZoneRight - 180;
		}
		Ri1 = ZoneRight - DeltaRight / 2;
		Ri2 = Ri1 + 360;

		if ((ZoneRight + DeltaRight / 2) > (ZoneForw - DeltaForw / 2) || (ZoneForw + DeltaForw / 2) > (ZoneLeft - DeltaLeft / 2) || (ZoneLeft + DeltaLeft / 2) > (ZoneBack - DeltaBack / 2) || (ZoneBack + DeltaBack / 2) > (Ri2))
		{
			Valid = 0;
			ZoneForw = 90;
			ZoneRight = 0;
			ZoneLeft = 180;
			ZoneBack = 270;
			DeltaForw = 20;
			DeltaRight = 60;
			DeltaLeft = 60;
			DeltaBack = 20;
		}

		//DeltaRot = 5;

		B_INDEX = 0; B_INDEX1 = 0; B_INDEX2 = 0;
		
		//INTERNAL USE
		int it;
		for (it = 0; it < 2 * NUMBER_OF_MOTIONS; it++)
			DIRECTION_ETIQUETTE[it] = ZERO;
		DIRECTION_ETIQUETTE[1] = FORWARD;	DIRECTION_ETIQUETTE[3] = BACKWARD;	 DIRECTION_ETIQUETTE[5] = RIGHT;	 DIRECTION_ETIQUETTE[7] = LEFT;
		DIRECTION_ETIQUETTE[9] = UP;		DIRECTION_ETIQUETTE[11] = DOWN;
		DIRECTION_ETIQUETTE[13] = FOR_RIGHT; DIRECTION_ETIQUETTE[15] = RIGHT_BACK; DIRECTION_ETIQUETTE[17] = BACK_LEFT; DIRECTION_ETIQUETTE[19] = LEFT_FOR;
	}

	void Control_HMI::process_payload(int *PCKT, char mapping_mode)
	{
		if (HEAD_IMU_STATE <= limit_missing)
			HEAD_IMU_STATE++;
		if (CHAIR_IMU_STATE <= limit_missing)
			CHAIR_IMU_STATE++;
		if (EMG_1_STATE <= limit_missing)
			EMG_1_STATE++;
		if (EMG_2_STATE <= limit_missing)
			EMG_2_STATE++;

		if (PCKT[0] == HEADSET)
		{
			HEAD_IMU_STATE = -1;
			HEAD_IMU.read_new_data(PCKT);
			HEAD_IMU.calibrate_imu();
			if (WBSN_MODE != WBSN_RECORDING)
				HEAD_IMU.rotate_imu();
			HEAD_IMU.sensor_fusion();
			if (WBSN_MODE != WBSN_RECORDING)
				algo(mapping_mode);
		}
		else if (PCKT[0] == IMU2) //REFERENCE)
		{
			CHAIR_IMU_STATE = -1;
			CHAIR_IMU.read_new_data(PCKT);
			CHAIR_IMU.calibrate_imu();
			CHAIR_IMU.sensor_fusion();
			algo(mapping_mode);
		}
		else if (PCKT[0] == IMU3)
		{
			IMU3_IMU_STATE = -1;
			IMU3_IMU.read_new_data(PCKT);
			IMU3_IMU.calibrate_imu();
			//IMU3_IMU.sensor_fusion();####
			//algo(mapping_mode);
		}
		else if (PCKT[0] == EMG1 || PCKT[0] == EMG)
		{
			EMG_1_STATE = -1;
			EMG_1.read_new_data(PCKT);
			EMG_1.filter_emg();
			EMG_1.process_tke();
			EMG_1.trigger_detect_tke(50);
		}
		else if (PCKT[0] == EMG2)
		{
			EMG_2_STATE = -1;
			EMG_2.read_new_data(PCKT);
			EMG_2.filter_emg();
			/*EMG_2.process_tke();
			EMG_2.trigger_detect_tke(50);*/
		}
		else if (PCKT[0] == EMG3)
		{
			EMG_3_STATE = -1;
			EMG_3.read_new_data(PCKT);
			EMG_3.filter_emg();
			/*EMG_3.process_tke();
			EMG_3.trigger_detect_tke(50);*/
		}
		else if (PCKT[0] == EMG4)
		{
			EMG_4_STATE = -1;
			EMG_4.read_new_data(PCKT);
			EMG_4.filter_emg();
			/*EMG_4.process_tke();
			EMG_4.trigger_detect_tke(50);*/
		}
	}

	int Control_HMI::update_calibration(void)
	{
		int res = 0;
		if (EMG_1.CALIB_EMG_INDEX == -2 || HEAD_IMU.CALIB_IMU_INDEX == -2)
		{
			res = 1;
			if (EMG_1.CALIB_EMG_INDEX == -2)
				EMG1_CALIBRATED = 1;

			EMG_1.CALIB_EMG_INDEX = -1;
			switch (HEAD_IMU.CALIB_TYPE)
			{
			case MAX_FORTH:
				AmaxForw = HEAD_IMU.amp_max_forw;
				AminForw = HEAD_IMU.amp_min_forw;
				ZoneForw = HEAD_IMU.zone_forw;
				YawCtrlForw = HEAD_IMU.yaw_ctrl_forw;
				imu_calibration[0] = 1;
				break;
			case MAX_BACK:
				AmaxBack = HEAD_IMU.amp_max_back;
				AminBack = HEAD_IMU.amp_min_back;
				ZoneBack = HEAD_IMU.zone_back;
				YawCtrlBack = HEAD_IMU.yaw_ctrl_back;
				imu_calibration[1] = 1;
				break;
			case MAX_RIGHT:
				AmaxRight = HEAD_IMU.amp_max_right;
				AminRight = HEAD_IMU.amp_min_right;
				ZoneRight = HEAD_IMU.zone_right;
				if (ZoneRight > 180)
				{
					ZoneRight = ZoneRight - 180;
				}
				Ri1 = ZoneRight - DeltaRight / 2;
				Ri2 = Ri1 + 360;
				YawCtrlRight = HEAD_IMU.yaw_ctrl_right;
				imu_calibration[2] = 1;
				break;
			case MAX_LEFT:
				AmaxLeft = HEAD_IMU.amp_max_left;
				AminLeft = HEAD_IMU.amp_min_left;
				ZoneLeft = HEAD_IMU.zone_left;
				YawCtrlLeft = HEAD_IMU.yaw_ctrl_left;
				imu_calibration[3] = 1;
				break;
			case MAX_ROT_RIGHT:
				RotmaxRight = HEAD_IMU.rot_max_right;
				RotminRight = HEAD_IMU.rot_min_right;
				PitchRotRight = HEAD_IMU.pitchrotright;
				RollRotRight = HEAD_IMU.rollrotright;
				AmpRotRight = HEAD_IMU.amp_rot_right;
				imu_calibration[4] = 1;
				break;
			case MAX_ROT_LEFT:
				RotmaxLeft = HEAD_IMU.rot_max_left;
				RotminLeft = HEAD_IMU.rot_min_left;
				PitchRotLeft = HEAD_IMU.pitchrotleft;
				RollRotLeft = HEAD_IMU.rollrotleft;
				AmpRotLeft = HEAD_IMU.amp_rot_left;
				imu_calibration[5] = 1;
				break;
			}
			HEAD_IMU.CALIB_TYPE = NO_CALIB;
			HEAD_IMU.CALIB_IMU_INDEX = -1;

			if ((ZoneRight + DeltaRight / 2) > (ZoneForw - DeltaForw / 2) || (ZoneForw + DeltaForw / 2) > (ZoneLeft - DeltaLeft / 2) || (ZoneLeft + DeltaLeft / 2) > (ZoneBack - DeltaBack / 2) || (ZoneBack + DeltaBack / 2) > (Ri2))
			{
				Valid = 0;
				ZoneForw = 90;
				ZoneRight = 0;
				ZoneLeft = 180;
				ZoneBack = 270;
				DeltaForw = 20;
				DeltaRight = 60;
				DeltaLeft = 60;
				DeltaBack = 20;
			}

			//CHECK IF HEADSET IMU IS CALIBRATED
			int nb_calib = 0;
			for (int ic = 0; ic < 6; ic++)//ic = 6 for 3D control
				nb_calib = nb_calib + imu_calibration[ic];
			if (nb_calib == 6 && HEAD_IMU.pitchoffset != 0) //nb_calib = 6 for 3D control
			{
				IMU_CALIBRATED = 1;
			}
		}
		return res;
	}

	void Control_HMI::algo(char mapping_mode)
	{
		DIRECTION = ZERO;
		x0 = HEAD_IMU.rollraw;
		y0 = HEAD_IMU.pitchraw;
		z0 = HEAD_IMU.yawraw;
		z0mean = HEAD_IMU.yawrawmean;
		z0_ref = CHAIR_IMU.yawraw;

		y = HEAD_IMU.pitchctrl;
		x = HEAD_IMU.rollctrl;
		z_ctrl = HEAD_IMU.yawctrl;

		// Polar Fusion
		Amp = 0.0; Theta = 0.0;
		if (fabsf(x) < 100 && fabsf(y) < 100)
		{
			Amp = sqrt(x*x + y*y);
			Theta = atan2(y, x) * 180 / M_PI;
		}

		z_2 = HEAD_IMU.yaw_ctrl[2];
		Amp_2 = 0.0; Theta_2 = 0.0;
		if (fabsf(z_2) < 100 && fabsf(y) < 100)
		{
			Amp_2 = sqrt(z_2*z_2 + y*y);
			Theta_2 = atan2(y, z_2) * 180 / M_PI;
		}
		
		float AmpF = 0; float CmdY = 0; float CmdX = 0;
		if (mapping_mode == PITCHROLL)
		{
			AmpF = Amp;
		}
		else if (mapping_mode== PITCHYAW)
		{
			AmpF = Amp_2;
			Theta = Theta_2;
		}
		// 0 .. 360 degrés ////////////////
		if (Theta < 0)
		{
			Theta = Theta + 360;
		}
		float Theta2 = Theta;
		if (Theta2 >= 180)
		{
			Theta2 = Theta2 - 360;
		}

		float Theta3 = Theta;
		if (Theta3 < 180)
		{
			Theta3 = Theta3 + 360;
		}
		//////////////////////////////////////////////////////////////////////
		
		float PortY = 0; float PortX = 0;
		//////////////////////////////////////////////////////////////////////
		if (Theta > ZoneForw - DeltaForw / 2 && Theta < ZoneForw + DeltaForw / 2)
		{
			//printf("FORWARD\n");
			DIRECTION = FORWARD;
			CmdY = AmpF;
		}
		else if (Theta > ZoneBack - DeltaBack / 2 && Theta < ZoneBack + DeltaBack / 2)
		{
			//printf("BACKWARD\n");
			DIRECTION = BACKWARD;
			CmdY = -AmpF;
		}
		else if (Theta > ZoneLeft - DeltaLeft / 2 && Theta < ZoneLeft + DeltaLeft / 2)
		{
			//printf("LEFT\n");
			DIRECTION = LEFT;
			CmdX = -AmpF;
		}
		else if (Theta < ZoneForw - DeltaForw / 2 && Theta > ZoneRight + DeltaRight / 2)
		{
			//printf("FOR_RIGHT\n");
			DIRECTION = FOR_RIGHT;
			if (DiagoActive == 1)
			{
				PortY = 1 - abs(Theta - (ZoneForw - DeltaForw / 2)) / abs((ZoneForw - DeltaForw / 2) - (ZoneRight + DeltaRight / 2));
				PortX = 1 - abs(Theta - (ZoneRight + DeltaRight / 2)) / abs((ZoneForw - DeltaForw / 2) - (ZoneRight + DeltaRight / 2));
				CmdY = AmpF * PortY;
				CmdX = AmpF * PortX;
			}
			else
			{
				DIRECTION = ZERO;
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta < ZoneLeft - DeltaLeft / 2 && Theta > ZoneForw + DeltaForw / 2)
		{
			//printf("LEFT_FOR\n");
			DIRECTION = LEFT_FOR;
			if (DiagoActive == 1)
			{
				PortX = 1 - abs(Theta - (ZoneLeft - DeltaLeft / 2)) / abs((ZoneLeft - DeltaLeft / 2) - (ZoneForw + DeltaForw / 2));
				PortY = 1 - abs(Theta - (ZoneForw + DeltaForw / 2)) / abs((ZoneLeft - DeltaLeft / 2) - (ZoneForw + DeltaForw / 2));
				CmdY = AmpF * PortY;
				CmdX = -AmpF * PortX;
			}
			else
			{
				DIRECTION = ZERO;
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta < ZoneBack - DeltaBack / 2 && Theta > ZoneLeft + DeltaLeft / 2)
		{
			//printf("BACK_LEFT\n");
			DIRECTION = BACK_LEFT;
			if (DiagoActive == 1)
			{
				PortY = 1 - abs(Theta - (ZoneBack - DeltaBack / 2)) / abs((ZoneBack - DeltaBack / 2) - (ZoneLeft + DeltaLeft / 2));
				PortX = 1 - abs(Theta - (ZoneLeft + DeltaLeft / 2)) / abs((ZoneBack - DeltaBack / 2) - (ZoneLeft + DeltaLeft / 2));
				CmdY = -AmpF * PortY;
				CmdX = -AmpF * PortX;
			}
			else
			{
				DIRECTION = ZERO;
				CmdY = 0;
				CmdX = 0;
			}
		}
		else if (Theta2 > Ri1 && Theta2 < (ZoneRight + DeltaRight / 2))
		{
			//printf("RIGHT\n");
			DIRECTION = RIGHT;
			CmdX = AmpF;
		}
		else if (Theta3 < Ri2 && Theta3 > ZoneBack + DeltaBack / 2)
		{
			//printf("RIGHT_BACK\n");
			DIRECTION = RIGHT_BACK;
			if (DiagoActive == 1)
			{
				PortX = 1 - abs(Theta3 - (Ri2)) / abs((Ri2)-(ZoneBack + DeltaBack / 2));
				PortY = 1 - abs(Theta3 - (ZoneBack + DeltaBack / 2)) / abs((Ri2)-(ZoneBack + DeltaBack / 2));
				CmdY = -AmpF * PortY;
				CmdX = AmpF * PortX;
			}
			else
			{
				DIRECTION = ZERO;
				CmdY = 0;
				CmdX = 0;
			}
		}

		if (abs(CmdX) <= AminBack && abs(CmdX) <= AminForw && abs(CmdY) <= AminRight && abs(CmdZ) <= AminLeft)
			DIRECTION = ZERO;

		CmdXout = 0;
		if (CmdX > AmaxBack)// - 0.15)			//BACK
		{
			CmdXout = 1;
		}
		else if (CmdX < -AmaxForw)// + 0.15)	//FORWARD
		{
			CmdXout = -1;
		}
		else if (CmdX >= 0)					//BACK
		{
			//if (CmdX <= AminBack)
				//DIRECTION = ZERO;
			CmdXout = ((CmdX - AminBack) / (AmaxBack - AminBack));// +0.15;
		}
		else if (CmdX < 0)					//FORWARD
		{
			//if (CmdX >= AminForw)
				//DIRECTION = ZERO;
			CmdXout = ((CmdX + AminForw) / (AmaxForw - AminForw));// -0.15;
		}
		CmdXout = -CmdXout;

		CmdYout = 0;
		if (CmdY > AmaxRight)// - 0.15)		//RIGHT
		{
			CmdYout = 1;
		}
		else if (CmdY < -AmaxLeft)// + 0.15)	//LEFT
		{
			CmdYout = -1;
		}
		else if (CmdY >= 0)					//RIGHT
		{
			//if (CmdY <= AminRight)
				//DIRECTION = ZERO;
			CmdYout = ((CmdY - AminRight) / (AmaxRight - AminRight));// +0.15;
		}
		else if (CmdY < 0)				//LEFT
		{
			//if (CmdY >= AminLeft)
				//DIRECTION = ZERO;
			CmdYout = ((CmdY + AminLeft) / (AmaxLeft - AminLeft));//;- 0.15;
		}
		CmdYout = CmdYout;


		if (ReferenceActive && SENSOR_HEADING_NOT_IN_PROGRESS)
			Theta_offset = z0mean - z0_ref;

		CmdX_new = CmdXout * cos(HEAD_IMU.deg2rad(Theta_offset)) + CmdYout * sin(HEAD_IMU.deg2rad(Theta_offset));
		CmdY_new = CmdXout * -sin(HEAD_IMU.deg2rad(Theta_offset)) + CmdYout * cos(HEAD_IMU.deg2rad(Theta_offset));

		// Commande Z
		//CmdZ = 0;
		if (ZActive == 1 && InhibiteZctrl == 0 && fabsf(z_ctrl) < 100 && ((z_ctrl < 0 && Amp <= (AmpRotRight + 6)) || (z_ctrl > 0 && Amp <= (AmpRotLeft + 6))))
		{
			//printf("ici");
			cpt_neutral++;
			if (cpt_neutral > 10) //ensures the person is in neutral mode not transiting though the neutral position
			{
				if (z_ctrl <= RotminLeft && z_ctrl >= RotminRight)
				{
					CmdZ = 0;
					CmdZ_new = CmdZ;
					//printf("NEUTRAL YAW\n");
				}
				else if (z_ctrl > RotminLeft && z_ctrl < RotmaxLeft)
				{
					//printf("DOWN\n");
					DIRECTION = DOWN;
					/*CmdZ = (z_ctrl - RotminLeft) / (RotmaxLeft - RotminLeft);
					CmdZ_new = CmdZ + 0.15;
					printf("LEFT\n");*/
					//CmdZ += 0.1;
					//if (CmdZ >(z_ctrl - RotminLeft) / (RotmaxLeft - RotminLeft))
					CmdZ = (z_ctrl - RotminLeft) / (RotmaxLeft - RotminLeft);
					CmdZ_new = CmdZ + 0.15;
					if (CmdZ_new > 1)
						CmdZ_new = 1;
				}
				else if (z_ctrl >= RotmaxLeft)
				{
					//printf("DOWN\n");
					DIRECTION = DOWN;
					/*CmdZ = (RotmaxLeft - RotminLeft) / (RotmaxLeft - RotminLeft);
					CmdZ_new = CmdZ;
					printf("LEFT\n");*/
					//CmdZ += 0.1;
					//if (CmdZ >= 1)
					CmdZ = 1;
					CmdZ_new = CmdZ;// + 0.15;
					if (CmdZ_new > 1)
						CmdZ_new = 1;
				}
				else if (z_ctrl < RotminRight && z_ctrl > RotmaxRight)
				{
					//printf("UP\n");
					DIRECTION = UP;
					/*CmdZ = -1 * (RotminRight - z_ctrl) / (RotminRight - RotmaxRight);
					CmdZ_new = CmdZ - 0.15;
					printf("RIGHT\n");*/
					//CmdZ -= 0.1;
					//if (CmdZ < -1 * (RotminRight - z_ctrl) / (RotminRight - RotmaxRight))
					CmdZ = -1 * (RotminRight - z_ctrl) / (RotminRight - RotmaxRight);
					CmdZ_new = CmdZ - 0.15;
					if (CmdZ_new < -1)
						CmdZ_new = -1;
				}
				else if (z_ctrl <= RotmaxRight)
				{
					//printf("UP\n");
					DIRECTION = UP;
					/*CmdZ = -1 * (RotminRight - RotmaxRight) / (RotminRight - RotmaxRight);
					CmdZ_new = CmdZ;
					printf("RIGHT\n");*/
					//CmdZ -= 0.1;
					//if (CmdZ < -1)
					CmdZ = -1;
					CmdZ_new = CmdZ;// - 0.15;
					if (CmdZ_new < -1)
						CmdZ_new = -1;
				}
			}
			else
			{
				CmdZ = 0;
				CmdZ_new = 0;
				//printf("NEUTRAL YAW\n");
			}
		}
		else
		{
			cpt_neutral = 0;//used to ensure the person is in neutral mode not transiting though the neutral position
			CmdZ = 0;
			CmdZ_new = 0;
			//printf("NEUTRAL YAW\n");
		}

		UpDownComm = CmdZ_new;
		RightLeftComm = -CmdX_new;
		ForBackComm = -CmdY_new;
	}
#ifdef _WIN32
	JoystickCommand Control_HMI::PushButton(char buttonindex)
	{
		JoystickCommand button_Command;
		button_Command.InitStruct();
		button_Command.ButtonValue[buttonindex] = 1;
		//button_Command.ButtonValue[B_INDEX2] = 0;
		return button_Command;
	}

	JoystickCommand Control_HMI::ReleaseButton(char buttonindex)
	{
		JoystickCommand button_Command;
		button_Command.InitStruct();
		button_Command.ButtonValue[buttonindex] = 0;
		//button_Command.ButtonValue[B_INDEX2] = 0;
		return button_Command;
	}

	JoystickCommand Control_HMI::CommandJACO(int b)
	{
		JoystickCommand control_Command;
		control_Command.InitStruct();
		if (b == 1)
		{
			RightLeftComm_ctrl = -RightLeftComm;
			ForBackComm_ctrl = ForBackComm;
			UpDownComm_ctrl = UpDownComm;
		}
		else
		{
			RightLeftComm_ctrl = 0.0;
			ForBackComm_ctrl = 0.0;
			UpDownComm_ctrl = 0.0;
		}
		control_Command.InclineLeftRight = RightLeftComm_ctrl;
		control_Command.InclineForwardBackward = ForBackComm_ctrl;
		control_Command.Rotate = -UpDownComm_ctrl;
		return control_Command;
	}
#endif
	///////////////  R  A  S  P  B  I  A  N  ///////////////////
#ifndef _WIN32
	jaco_joystick_t Control_HMI::CommandJACO_RPI(int b)
	{
		jaco_joystick_t control_Command = {0};
		if (b == 1)
		{
			RightLeftComm_ctrl = -RightLeftComm;
			ForBackComm_ctrl = ForBackComm;
			UpDownComm_ctrl = UpDownComm;
		}
		else
		{
			RightLeftComm_ctrl = 0.0;
			ForBackComm_ctrl = 0.0;
			UpDownComm_ctrl = 0.0;
		}
		control_Command.axis.trans_lr = RightLeftComm_ctrl*2.5;
		control_Command.axis.trans_fb = ForBackComm_ctrl*2.5;
		control_Command.axis.trans_rot = UpDownComm_ctrl*2.5;
		return control_Command;
	}
#endif
	///////////////////////////////////////////////////////////

	float Control_HMI::fmaxf(float x, float y)
	{
		volatile float res = 0;
		if (x >= y)
			res =  x;
		else if (x < y)
			res = y;
		return res;
	}
#ifdef _WIN32
	int Control_HMI::ReadJACOMode()
	{
		int status_jaco, mode_jaco;
		mode_jaco = 0;
		static int(*MyGetControlMapping)(ControlMappingCharts &Response);
		ControlMappingCharts MappingJACO;
		status_jaco = (*MyGetControlMapping)(MappingJACO);
		mode_jaco = MappingJACO.ActualControlMapping;
		return mode_jaco;
	}
#endif
}
