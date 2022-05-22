#pragma once

#define		HEADSET_IMU		10
#define		REFERENCE_IMU	20

#include <math.h>
#include "Defined_Macro.h"

namespace Control_IMU_JACO
{
	class IMU_Sensor
	{
	public:

		char IMU_type;
		char SENSOR_ON;
		int FREQ;
		int PACKET_NUMBER;
		int LOST_PACKETS;

		//filters
		static const double num_lp_10Hz[3]; static const double den_lp_10Hz[3];
		static const double num_lp_25Hz[3]; static const double den_lp_25Hz[3];

		//calib
		double comp[3][3]; double hardiron[3];

		//data
		double init_rotmat[3][3];
		float acc[3];
		float gyr[6]; // 0:2 --> raw values, 3:5 --> calibrated values
		float mag[6]; // 0:2 --> raw values, 3:5 --> calibrated values
		float pitch[3];
		float x_pitch[3];
		float roll[3];
		float x_roll[3];
		float yaw[3];
		float x_yaw[3];
		float xh_cal, yh_cal, zh_cal;
		float pitchraw, rollraw, yawraw, yawrawmean;

		//control
		float pitchoffset, rolloffset, yawoffset;
		float pitchctrl, rollctrl, yawctrl;
		float xh_ctrl, yh_ctrl, zh_ctrl;
		float yaw_ctrl[3];
		float x_yaw_ctrl[3];

		//calib
		static const int NB_VALUES_CALIB = 2000;//7.5s //100;//1.5s// 3;//50ms // //  
		int NB_VALUES_CALIB_CURRENT;
		int CALIB_IMU_INDEX;
		float pitch_calib[NB_VALUES_CALIB];
		float  roll_calib[NB_VALUES_CALIB];
		float   yaw_calib[NB_VALUES_CALIB];
		
		int CALIB_TYPE;
		float amp_max_forw, amp_max_back, amp_max_right, amp_max_left;
		float yaw_ctrl_forw, yaw_ctrl_back, yaw_ctrl_right, yaw_ctrl_left;
		float amp_min_forw, amp_min_back, amp_min_right, amp_min_left;
		float rot_min_right, rot_max_right, rot_min_left, rot_max_left, pitchrotright, rollrotright, pitchrotleft, rollrotleft, amp_rot_right, amp_rot_left;
		float zone_forw, zone_back, zone_right, zone_left;
		float delta_forw, delta_back, delta_right, delta_left;
		float port_min_max;
		
		float acc_calib[NB_VALUES_CALIB]; //acc along the vertical axis in neutral position
		float mean_acc_calib;
		float margin_acc_calib; //mean acc margin along the vertical axis in neutral position
		float max_acc_calib;
		float min_acc_calib;
		
		static const int nb_mean_acc = 50;
		int	nb_overlap_mean_acc;
		float acc_flat_yaw_tab[nb_mean_acc]; int i_nb_acc;
		float mean_acc_flat_yaw; //current mean acc along the vertical axis in neutral position
		

		//reference : velocity dectection on x
		static const int nb_val_gyr = 30;
		float moy_gyr[nb_val_gyr];
		int index_gyr;
		float MeanVelYaw;
		float ThRotReference;
		char rotation_detected;

		IMU_Sensor();
		~IMU_Sensor();
		void initialize(char type);
		void read_new_data(int data[]);
		void check_lost_packets(int pckt_nb);
		void rotate_imu();
		void calibrate_imu(void);
		// INPUT [3]: pitchoff, pitchyoff, pitchzoff (clibration) | OUTPUT [4]: (raw-offset) : shifted pitch, roll, yaw, chair rotation indicator 
		void sensor_fusion(void);
		char detect_wheelchair_rotation();
		float mean_tabf(float * tab, int taille);
		float min_tabf(float * tab, int taille);
		float max_tabf(float * tab, int taille);
		float C2_2(int var);
		float rad2deg(float angle_rad);
		float deg2rad(float angle_deg);
		float absf(float x);
	};

}