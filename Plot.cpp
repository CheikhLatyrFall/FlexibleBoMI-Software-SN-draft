#include "Plot.h"

namespace Control_IMU_JACO
{
	Plot::Plot()
	{}

	Plot::~Plot()
	{}

	void Plot::init_plot_variables(char EMG_PRECISION, Control_HMI * ctrl_main, char MODE_PLOT)
	{
		ctrl_jaco = ctrl_main;
		EMG_SAMPLES_PRECISION = EMG_PRECISION;
		if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
			NB_SAMPLES_BY_PCKT = 13;
		else if (EMG_SAMPLES_PRECISION == EMG_24BIT_PRECISION)
			NB_SAMPLES_BY_PCKT = 9;
		NB_SAMPLES_TO_PLOT = 7 * NB_SAMPLES_BY_PCKT;
		NB_SCOPE_SAMPLES = 30 * NB_SAMPLES_TO_PLOT;
		NB_TKE_TO_PLOT = 15;
		NB_SCOPE_TKE = 50 * NB_TKE_TO_PLOT;
		
		NB_IMU_TO_PLOT = 5;
		NB_SCOPE_IMU = 50 * NB_IMU_TO_PLOT;

		NB_IMU_TO_PLOT2 = 5;
		NB_SCOPE_IMU2 = 50 * NB_IMU_TO_PLOT2;

		NB_IMU_TO_PLOT3 = 5;
		NB_SCOPE_IMU3 = 50 * NB_IMU_TO_PLOT3;

		polar_calib_set = 1;

		switch (MODE_PLOT)
		{
		case PLOT_SENTINELLE:
			i_nemg = 0;
			i_ntke = 0;
			th = 0;
			new_emg = new float[NB_SAMPLES_TO_PLOT];
			new_tke = new float[NB_TKE_TO_PLOT];
			break;
		case PLOT_CTRL:
			i_nemg = 0;
			i_n2emg = 0;
			i_ntke = 0;
			th = 0;
			new_emg = new float[NB_SAMPLES_TO_PLOT];
			new2_emg = new float[NB_SAMPLES_TO_PLOT];
			new_tke = new float[NB_TKE_TO_PLOT];
			break;
		case PLOT_MULTI_EMG:
			i_nch1 = 0; i_nch2 = 0; i_nch3 = 0; i_nch4 = 0;
			new_ch1 = new float[NB_SAMPLES_TO_PLOT];
			new_ch2 = new float[NB_SAMPLES_TO_PLOT];
			new_ch3 = new float[NB_SAMPLES_TO_PLOT];
			new_ch4 = new float[NB_SAMPLES_TO_PLOT];
			break;
		case PLOT_2D_POLAR:
			i_nemg = 0;
			i_n2emg = 0;
			i_ntke = 0;
			th = 0;
			new_emg = new float[NB_SAMPLES_TO_PLOT];
			new2_emg = new float[NB_SAMPLES_TO_PLOT];
			new_tke = new float[NB_TKE_TO_PLOT];
			break;
		default:
			break;
		}
	}

	void Plot::set_charts(char MODE_PLOT)
	{
		if (MODE_PLOT != OFF)
			PLOT_ACTIVE = 1;
		MODE = MODE_PLOT;

		switch (MODE)
		{
		case PLOT_SENTINELLE:
			PLOT_1.emg_precision = EMG_SAMPLES_PRECISION;
			//PLOT_1.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2);
			PLOT_1.initialize(IMU_SCOPE, 1, NB_SCOPE_IMU,  NB_IMU_TO_PLOT, 2);
			PLOT_2.initialize(IMU_SCOPE, 2, NB_SCOPE_IMU2, NB_IMU_TO_PLOT2, 2);
			PLOT_3.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU3, NB_IMU_TO_PLOT3, 2); 
			break;

		case PLOT_CTRL:
			PLOT_1.emg_precision = EMG_SAMPLES_PRECISION;
			PLOT_1.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2); //Livrable 11
			PLOT_2.initialize(TKE_SCOPE, 2, NB_SCOPE_TKE, NB_TKE_TO_PLOT, 2);
			PLOT_3.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU, NB_IMU_TO_PLOT, 2); //Livrable 11
			break;
		
		case PLOT_MULTI_EMG:
			PLOT_1.emg_precision = EMG_SAMPLES_PRECISION;
			PLOT_2.emg_precision = EMG_SAMPLES_PRECISION;
			PLOT_3.emg_precision = EMG_SAMPLES_PRECISION;
			PLOT_1.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 1);
			PLOT_2.initialize(EMG_SCOPE, 2, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 1);
			PLOT_3.initialize(EMG_SCOPE, 3, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 1);
			break;
		
		case PLOT_2D_POLAR:
			PLOT_1.emg_precision = EMG_SAMPLES_PRECISION;
			PLOT_1.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2);
			PLOT_2.initialize(POLAR_SCOPE, 2, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT);
			PLOT_3.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU, NB_IMU_TO_PLOT, 2);
			break;

		default:
			break;
		}
	}

	void Plot::update_plot(int sensor_data[])
	{
		if (PLOT_ACTIVE)
		{
			switch (MODE)
			{
			case PLOT_SENTINELLE:
				if (sensor_data[0] == IMU1)
				{
					new_imu[0] = (*ctrl_jaco).HEAD_IMU.pitchctrl;
					new_imu[1] = (*ctrl_jaco).HEAD_IMU.rollctrl;
					new_imu[2] = (*ctrl_jaco).HEAD_IMU.yaw_ctrl[2];
					PLOT_1.update(new_imu, 3, 1);
				}
				else if (sensor_data[0] == IMU2)
				{
					new_imu2[0] = (*ctrl_jaco).CHAIR_IMU.pitchctrl;
					new_imu2[1] = (*ctrl_jaco).CHAIR_IMU.rollctrl;
					new_imu2[2] = (*ctrl_jaco).CHAIR_IMU.yaw_ctrl[2];
					PLOT_2.update(new_imu2, 3, 1);
				}
				else if (sensor_data[0] == IMU3)
				{
					new_imu3[0] = (*ctrl_jaco).IMU3_IMU.pitchctrl;
					new_imu3[1] = (*ctrl_jaco).IMU3_IMU.rollctrl;
					new_imu3[2] = (*ctrl_jaco).IMU3_IMU.yaw_ctrl[2];
					PLOT_3.update(new_imu3, 3, 1);
				}
				else if (sensor_data[0] == EMG1 || sensor_data[0] == EMG)
				{
					if ((*ctrl_jaco).EMG_1.NEW_CALIB)
					{
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
						PLOT_1.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
						PLOT_1.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
					}
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = ((*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_emg[i_nemg + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_emg[i_nemg + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_emg[i_nemg + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_emg[i_nemg + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
						else if (EMG_SAMPLES_PRECISION == EMG_24BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = (*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg];
							if (new_emg[i_nemg + pemg] >= 100)
								new_emg[i_nemg + pemg] = 100;
							else if (new_emg[i_nemg + pemg] <= -100)
								new_emg[i_nemg + pemg] = -100;
						}
					}
					i_nemg += NB_SAMPLES_BY_PCKT;

					//new_tke[i_ntke] = (*ctrl_jaco).EMG_1.MEAN_TKE_CURRENT;
					//i_ntke += 1;

					if (i_nemg >= PLOT_1.nb_data_to_plot)
					{
						PLOT_1.update(new_emg, PLOT_1.nb_data_to_plot, 1);
						i_nemg = 0;
					}
					/*if (i_ntke >= PLOT_2.nb_data_to_plot)
					{
					PLOT_2.update(new_tke, PLOT_2.nb_data_to_plot, 1);
					PLOT_2.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
					PLOT_2.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
					i_ntke = 0;
					}*/
				}
				break;
			case PLOT_CTRL:
				if (sensor_data[0] == HEADSET)
				{
					new_imu[0] = (*ctrl_jaco).HEAD_IMU.pitchctrl;
					new_imu[1] = (*ctrl_jaco).HEAD_IMU.rollctrl;
					new_imu[2] = (*ctrl_jaco).HEAD_IMU.yawctrl;
					PLOT_3.update(new_imu, 3, 1);
				}
				else if (sensor_data[0] == REFERENCE)
				{
				}
				else if (sensor_data[0] == EMG1 || sensor_data[0] == EMG)
				{
					if ((*ctrl_jaco).EMG_1.NEW_CALIB)
					{
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
						PLOT_2.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
						PLOT_2.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
					}
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = ((*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_emg[i_nemg + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_emg[i_nemg + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_emg[i_nemg + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_emg[i_nemg + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
						else if (EMG_SAMPLES_PRECISION == EMG_24BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = (*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg];
							if (new_emg[i_nemg + pemg] >= 100)
								new_emg[i_nemg + pemg] = 100;
							else if (new_emg[i_nemg + pemg] <= -100)
								new_emg[i_nemg + pemg] = -100;
						}
					}
					i_nemg += NB_SAMPLES_BY_PCKT;

					new_tke[i_ntke] = (*ctrl_jaco).EMG_1.MEAN_TKE_CURRENT;
					i_ntke += 1; 

					if (i_nemg >= PLOT_1.nb_data_to_plot)
					{
						PLOT_1.update(new_emg, PLOT_1.nb_data_to_plot, 1);
						i_nemg = 0;
					}
					if (i_ntke >= PLOT_2.nb_data_to_plot)
					{
						PLOT_2.update(new_tke, PLOT_2.nb_data_to_plot, 1);
						PLOT_2.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
						PLOT_2.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
						i_ntke = 0;
					}
				}

				/*else if (sensor_data[0] == EMG2)
				{
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						new2_emg[i_n2emg + pemg] = ((*ctrl_jaco).EMG_2.DATA_EMG_FILT[pemg] + CH2_POS)*PRECISION_16BIT_SCALE;
						if (new2_emg[i_n2emg + pemg] >= (400 + CH2_POS)*PRECISION_16BIT_SCALE)
							new2_emg[i_n2emg + pemg] = (400 + CH2_POS)*PRECISION_16BIT_SCALE;
						else if (new2_emg[i_n2emg + pemg] <= (-400 + CH2_POS)*PRECISION_16BIT_SCALE)
							new2_emg[i_n2emg + pemg] = (-400 + CH2_POS)*PRECISION_16BIT_SCALE;
					}
					i_n2emg += NB_SAMPLES_BY_PCKT;

					if (i_n2emg >= PLOT_EMG.nb_data_to_plot)
					{
						PLOT_EMG.update(new2_emg, PLOT_EMG.nb_data_to_plot, 2);
						i_n2emg = 0;
					}
				}*/
				break;

			case PLOT_MULTI_EMG:
				if (sensor_data[0] == EMG1)
				{
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_ch1[i_nch1 + pemg] = ((*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_ch1[i_nch1 + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_ch1[i_nch1 + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_ch1[i_nch1 + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_ch1[i_nch1 + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
					}
					i_nch1 += NB_SAMPLES_BY_PCKT;

					if (i_nch1 >= PLOT_1.nb_data_to_plot)
					{
						PLOT_1.update(new_ch1, PLOT_1.nb_data_to_plot, 1);
						i_nch1 = 0;
					}
				}
				
				else if (sensor_data[0] == EMG2)
				{
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_ch2[i_nch2 + pemg] = ((*ctrl_jaco).EMG_2.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_ch2[i_nch2 + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_ch2[i_nch2 + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_ch2[i_nch2 + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_ch2[i_nch2 + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
					}
					i_nch2 += NB_SAMPLES_BY_PCKT;

					if (i_nch2 >= PLOT_2.nb_data_to_plot)
					{
						PLOT_2.update(new_ch2, PLOT_2.nb_data_to_plot, 1);
						i_nch2 = 0;
					}
				}

				else if (sensor_data[0] == EMG3)
				{
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_ch3[i_nch3 + pemg] = ((*ctrl_jaco).EMG_3.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_ch3[i_nch3 + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_ch3[i_nch3 + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_ch3[i_nch3 + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_ch3[i_nch3 + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
					}
					i_nch3 += NB_SAMPLES_BY_PCKT;

					if (i_nch3 >= PLOT_3.nb_data_to_plot)
					{
						PLOT_3.update(new_ch3, PLOT_3.nb_data_to_plot, 1);
						i_nch3 = 0;
					}
				}

				else if(sensor_data[0] == EMG4)
				{
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_ch4[i_nch4 + pemg] = ((*ctrl_jaco).EMG_4.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_ch4[i_nch4 + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_ch4[i_nch4 + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_ch4[i_nch4 + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_ch4[i_nch4 + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
					}
					i_nch4 += NB_SAMPLES_BY_PCKT;

					if (i_nch4 >= PLOT_4.nb_data_to_plot)
					{
						PLOT_4.update(new_ch4, PLOT_4.nb_data_to_plot, 1);
						i_nch4 = 0;
					}
				}

				break;

			case PLOT_2D_POLAR:
				if (sensor_data[0] == HEADSET)
				{
					new_imu[0] = (*ctrl_jaco).HEAD_IMU.pitchctrl;
					new_imu[1] = (*ctrl_jaco).HEAD_IMU.rollctrl;
					new_imu[2] = (*ctrl_jaco).HEAD_IMU.yawctrl;
					PLOT_3.update(new_imu, 3, 1); //Livrable 11
					new_imu[0] = (*ctrl_jaco).Amp/90; //make sure between 0 and 1
					new_imu[1] = (*ctrl_jaco).Theta * M_PI/180; //make sure in radian
					PLOT_2.update(new_imu, 2);
					if (polar_calib_set == 0)
					{
						int ir;
						float step_phi_right = (*ctrl_jaco).DeltaRight / 50; float step_phi_left = (*ctrl_jaco).DeltaLeft / 50;
						float step_phi_forw  = (*ctrl_jaco).DeltaForw  / 50; float step_phi_back = (*ctrl_jaco).DeltaBack / 50;

						float start_phi_right = (*ctrl_jaco).ZoneRight - (*ctrl_jaco).DeltaRight / 2; float start_phi_left = (*ctrl_jaco).ZoneLeft - (*ctrl_jaco).DeltaLeft / 2;
						float start_phi_forw  = (*ctrl_jaco).ZoneForw  - (*ctrl_jaco).DeltaForw  / 2; float start_phi_back = (*ctrl_jaco).ZoneBack - (*ctrl_jaco).DeltaBack / 2;
						
						float step_r_right = ((*ctrl_jaco).AmaxRight - (*ctrl_jaco).AminRight) / 50;  float step_r_left = ((*ctrl_jaco).AmaxLeft - (*ctrl_jaco).AminLeft) / 50;
						float step_r_forw  = ((*ctrl_jaco).AmaxForw  - (*ctrl_jaco).AminForw)  / 50;  float step_r_back = ((*ctrl_jaco).AmaxBack - (*ctrl_jaco).AminBack) / 50;
						
						for (ir = 0; ir < 50; ir++)
						{
							PLOT_2.r_min[ir] = (*ctrl_jaco).AminRight / 90;      PLOT_2.r_max[ir] = (*ctrl_jaco).AmaxRight / 90;
							PLOT_2.r_min[50  + ir] = (*ctrl_jaco).AminLeft / 90; PLOT_2.r_max[50  + ir] = (*ctrl_jaco).AmaxLeft / 90;
							PLOT_2.r_min[100 + ir] = (*ctrl_jaco).AminForw / 90; PLOT_2.r_max[100 + ir] = (*ctrl_jaco).AmaxForw / 90;
							PLOT_2.r_min[150 + ir] = (*ctrl_jaco).AminBack / 90; PLOT_2.r_max[150 + ir] = (*ctrl_jaco).AmaxBack / 90;

							PLOT_2.phi_calib[ir] = (start_phi_right + ir*step_phi_right) * M_PI / 180; PLOT_2.phi_calib[50 + ir] = (start_phi_left + ir*step_phi_left) * M_PI / 180;
							PLOT_2.phi_calib[100 + ir] = (start_phi_forw + ir*step_phi_forw) * M_PI / 180; PLOT_2.phi_calib[150 + ir] = (start_phi_back + ir*step_phi_back) * M_PI / 180;

							PLOT_2.r_border[ir] = ((*ctrl_jaco).AminRight + ir*step_r_right) / 90; PLOT_2.r_border[50 + ir] = ((*ctrl_jaco).AminLeft + ir*step_r_left) / 90;
							PLOT_2.r_border[100 + ir] = ((*ctrl_jaco).AminForw  + ir*step_r_forw) / 90; PLOT_2.r_border[150 + ir] = ((*ctrl_jaco).AminBack + ir*step_r_back) / 90;

							PLOT_2.phi_border[ir] = start_phi_right * M_PI / 180;      PLOT_2.phi_border[50 + ir] = (start_phi_right + (*ctrl_jaco).DeltaRight) * M_PI / 180;
							PLOT_2.phi_border[100 + ir] = start_phi_left * M_PI / 180; PLOT_2.phi_border[150 + ir] = (start_phi_left + (*ctrl_jaco).DeltaLeft)  * M_PI / 180;
							PLOT_2.phi_border[200 + ir] = start_phi_forw * M_PI / 180; PLOT_2.phi_border[250 + ir] = (start_phi_forw + (*ctrl_jaco).DeltaForw)  * M_PI / 180;
							PLOT_2.phi_border[300 + ir] = start_phi_back * M_PI / 180; PLOT_2.phi_border[350 + ir] = (start_phi_back + (*ctrl_jaco).DeltaBack)  * M_PI / 180;
						}
						PLOT_2.diag.color("white");
						//right
						PLOT_2.diag.curve(PLOT_2.r_min,PLOT_2.phi_calib,50);
						PLOT_2.diag.curve(PLOT_2.r_max, PLOT_2.phi_calib, 50);
						PLOT_2.diag.curve(PLOT_2.r_border, PLOT_2.phi_border, 50);
						PLOT_2.diag.curve(PLOT_2.r_border, PLOT_2.phi_border + 50, 50);
						//left
						PLOT_2.diag.curve(PLOT_2.r_min+50, PLOT_2.phi_calib+50, 50);
						PLOT_2.diag.curve(PLOT_2.r_max+50, PLOT_2.phi_calib+50, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+50, PLOT_2.phi_border+100, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+50, PLOT_2.phi_border+150, 50);
						//forw
						PLOT_2.diag.curve(PLOT_2.r_min+100, PLOT_2.phi_calib+100, 50);
						PLOT_2.diag.curve(PLOT_2.r_max+100, PLOT_2.phi_calib+100, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+100, PLOT_2.phi_border+200, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+100, PLOT_2.phi_border+250, 50);
						//back
						PLOT_2.diag.curve(PLOT_2.r_min+150, PLOT_2.phi_calib+150, 50);
						PLOT_2.diag.curve(PLOT_2.r_max+150, PLOT_2.phi_calib+150, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+150, PLOT_2.phi_border+300, 50);
						PLOT_2.diag.curve(PLOT_2.r_border+150, PLOT_2.phi_border+350, 50);
						polar_calib_set = 1;
					}
				}
				else if (sensor_data[0] == EMG1 || sensor_data[0] == EMG)
				{
					/*if ((*ctrl_jaco).EMG_1.NEW_CALIB)
					{
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
						PLOT_2.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
						PLOT_2.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
						(*ctrl_jaco).EMG_1.NEW_CALIB = 0;
					}*/
					for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
					{
						if (EMG_SAMPLES_PRECISION == EMG_16BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = ((*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg] + CH1_POS_CTRL)*PRECISION_16BIT_SCALE; //+ 150;
							if (new_emg[i_nemg + pemg] >= (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//100+150)
								new_emg[i_nemg + pemg] = (400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//100+150;
							else if (new_emg[i_nemg + pemg] <= (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE)//-100+150)
								new_emg[i_nemg + pemg] = (-400 + CH1_POS_CTRL)*PRECISION_16BIT_SCALE;//-100+150;
						}
						else if (EMG_SAMPLES_PRECISION == EMG_24BIT_PRECISION)
						{
							new_emg[i_nemg + pemg] = (*ctrl_jaco).EMG_1.DATA_EMG_FILT[pemg];
							if (new_emg[i_nemg + pemg] >= 100)
								new_emg[i_nemg + pemg] = 100;
							else if (new_emg[i_nemg + pemg] <= -100)
								new_emg[i_nemg + pemg] = -100;
						}
					}
					i_nemg += NB_SAMPLES_BY_PCKT;

					/*new_tke[i_ntke] = (*ctrl_jaco).EMG_1.MEAN_TKE_CURRENT;
					i_ntke += 1;*/

					if (i_nemg >= PLOT_1.nb_data_to_plot)
					{
						PLOT_1.update(new_emg, PLOT_1.nb_data_to_plot, 1);
						i_nemg = 0;
					}
					/*if (i_ntke >= PLOT_2.nb_data_to_plot)
					{
						PLOT_2.update(new_tke, PLOT_2.nb_data_to_plot, 1);
						PLOT_2.update((*ctrl_jaco).EMG_1.THRESHOLD, PLOT_2.nb_data_to_plot, 2);
						PLOT_2.update((*ctrl_jaco).EMG_1.UPPER_THRESHOLD, (*ctrl_jaco).EMG_1.LOWER_THRESHOLD, PLOT_2.nb_data_to_plot, 3);
						i_ntke = 0;
					}*/
				}
				break;
			}
		}
	}
}