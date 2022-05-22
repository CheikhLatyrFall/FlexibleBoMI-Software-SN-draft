#pragma once

#include "Dislin_Scope.h"
#include "Control_HMI.h"

namespace Control_IMU_JACO
{
	class Plot
	{
	public:

		bool PLOT_ACTIVE;
		char MODE;
		char EMG_SAMPLES_PRECISION;
		Control_HMI * ctrl_jaco;

		Dislin_Scope PLOT_1;
		Dislin_Scope PLOT_2;
		Dislin_Scope PLOT_3;
		Dislin_Scope PLOT_4;
		
		bool polar_calib_set;

		int i_nch1, i_nch2, i_nch3, i_nch4;
		float * new_ch1 = NULL; float * new_ch2 = NULL;
		float * new_ch3 = NULL; float * new_ch4 = NULL;

		int i_nemg; int i_n2emg;
		float * new_emg = NULL;
		float * new2_emg = NULL;
		int NB_SAMPLES_BY_PCKT; int NB_SAMPLES_TO_PLOT; int NB_SCOPE_SAMPLES;

		int i_ntke;
		float * new_tke = NULL;
		float th;
		int NB_TKE_TO_PLOT; int NB_SCOPE_TKE;

		float new_imu[4];
		int NB_IMU_TO_PLOT; int NB_SCOPE_IMU;

		float new_imu2[4];
		int NB_IMU_TO_PLOT2; int NB_SCOPE_IMU2;

		float new_imu3[4];
		int NB_IMU_TO_PLOT3; int NB_SCOPE_IMU3;

		Plot();
		~Plot();
		void init_plot_variables(char EMG_PRECISION, Control_HMI * ctrl_main, char MODE_PLOT);
		void set_charts(char MODE_PLOT);
		void update_plot(int sensor_data[]);
	};
}

