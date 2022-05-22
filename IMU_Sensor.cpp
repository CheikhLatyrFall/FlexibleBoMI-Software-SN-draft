#include "IMU_Sensor.h"

namespace Control_IMU_JACO
{
	const double IMU_Sensor::num_lp_10Hz[3] = { 0.355, 0.355, 0.0 };
	const double IMU_Sensor::num_lp_25Hz[3] = { 0.755, 0.755, 0.0 };
	const double IMU_Sensor::den_lp_10Hz[3] = { 1.0, -0.2905, 0.0 };
	const double IMU_Sensor::den_lp_25Hz[3] = { 1.0, 0.5095, 0.0 };


	IMU_Sensor::IMU_Sensor()
	{}


	IMU_Sensor::~IMU_Sensor()
	{}


	void IMU_Sensor::initialize(char type)
	{
		IMU_type = 0;
		ThRotReference = 30;
		rotation_detected = 0;

		PACKET_NUMBER = 0;
		LOST_PACKETS = 0;
		pitchoffset = 0; rolloffset = 0; yawoffset = 0; yawrawmean = 500;
		amp_max_forw = 0; amp_max_back = 0; amp_max_right = 0; amp_max_left = 0;
		amp_min_forw = 0; amp_min_back = 0; amp_min_right = 0; amp_min_left = 0;
		yaw_ctrl_forw = 0; yaw_ctrl_back = 0; yaw_ctrl_right = 0; yaw_ctrl_left = 0;
		rot_max_right = 0; rot_max_left = 0; rot_min_right = 0; rot_min_left = 0;
		pitchrotright = 0; rollrotright = 0; pitchrotleft = 0; rollrotleft = 0;
		amp_rot_right = 0; amp_rot_right = 0;
		port_min_max = 0.1;

		mean_acc_calib = 0; margin_acc_calib = 0; 
		max_acc_calib = 0;  min_acc_calib = 0;
		mean_acc_flat_yaw = 0;
		i_nb_acc = 0;
		nb_overlap_mean_acc = 20;

		CALIB_IMU_INDEX = -1; CALIB_TYPE = 0;
		
		if (type == HEADSET_IMU)
		{
			IMU_type = HEADSET_IMU;
			comp[0][0] = 1.6357e-4;  comp[0][1] = -1.0404e-5; comp[0][2] = -1.4933e-6;
			comp[1][0] = -1.0404e-5; comp[1][1] = 1.5810e-4;  comp[1][2] = 6.6342e-7;
			comp[2][0] = 1.4933e-6;  comp[2][1] = 6.6342e-7;  comp[2][2] = 1.6295e-4;

			hardiron[0] = -127.4200; hardiron[1] = 158.6824; hardiron[2] = -57.7485;
		}
		else if (type == IMU2)
		{
			IMU_type = IMU2;
			comp[0][0] = 1.444290812432278e-04;  comp[0][1] = -1.206751868374715e-05; comp[0][2] = -1.485850436757718e-06;
			comp[1][0] = -1.206751868374715e-05; comp[1][1] = 1.424178843508972e-04;  comp[1][2] = -1.555098679464108e-06;
			comp[2][0] = -1.485850436757721e-06; comp[2][1] = -1.555098679464111e-06; comp[2][2] = 1.446113697529971e-04;

			hardiron[0] = -1.572530345324899e+03; hardiron[1] = 6.593662556267990; hardiron[2] = 1.947946912677864e+03;
		}
		else if (type == IMU3)
		{
			IMU_type = IMU3;
			comp[0][0] = 2.139025999407627e-04;  comp[0][1] = -1.988173408113560e-05; comp[0][2] = -3.519446757842074e-06;
			comp[1][0] = -1.988173408113563e-05; comp[1][1] = 2.141395882424159e-04;  comp[1][2] = -3.480400573194146e-06;
			comp[2][0] = -3.519446757842071e-06; comp[2][1] = -3.480400573194143e-06; comp[2][2] = 2.056425277269774e-04;

			hardiron[0] = 6.097267251407045e+02; hardiron[1] = -1.145313399686339e+03; hardiron[2] = 33.410171460617630;
		}
		
	}

	void IMU_Sensor::rotate_imu()
	{
		if (IMU_type == HEADSET)
		{
			float var0, var1, var2;
			var0 = acc[0]; var1 = acc[1]; var2 = acc[2];
			acc[0] = -1 * var2;
			acc[1] = -1 * var0;
			acc[2] = 1 * var1;

			var0 = gyr[0]; var1 = gyr[1]; var2 = gyr[2];
			gyr[0] = -1 * var2;
			gyr[1] = -1 * var0;
			gyr[2] = 1 * var1;

			var0 = mag[3]; var1 = mag[4]; var2 = mag[5];
			mag[3] = -1 * var2;
			mag[4] = -1 * var0;
			mag[5] = 1 * var1;
		}
	}

	void IMU_Sensor::read_new_data(int data[])
	{
		check_lost_packets(data[2]);
		SENSOR_ON = 1;
		if (data[1] != 0)
		{
			FREQ = 1000 / data[1];											// MSP430 Sensor Acquisition Frequency
		}

		acc[0] = C2_2((data[16] << 8) + data[17]);
		acc[1] = C2_2((data[18] << 8) + data[19]);
		acc[2] = C2_2((data[20] << 8) + data[21]);

		gyr[0] = C2_2((data[10] << 8) + data[11]);
		gyr[1] = C2_2((data[12] << 8) + data[13]);
		gyr[2] = C2_2((data[14] << 8) + data[15]);
		if (IMU_type == REFERENCE_IMU)
		{
			moy_gyr[index_gyr] = absf(gyr[0]);
			index_gyr = index_gyr + 1;
		}

		mag[0] = C2_2((data[4] << 8) + data[5]);
		mag[1] = C2_2((data[6] << 8) + data[7]);
		mag[2] = C2_2((data[8] << 8) + data[9]);
	}

	void IMU_Sensor::check_lost_packets(int pckt_nb)
	{
		volatile int var;
		if (PACKET_NUMBER < pckt_nb && pckt_nb <= 255 && SENSOR_ON)
			var = (pckt_nb - PACKET_NUMBER) - 1;
		else if (PACKET_NUMBER > pckt_nb && SENSOR_ON)
			var = (256 + pckt_nb - PACKET_NUMBER) - 1;

		PACKET_NUMBER = pckt_nb;
		LOST_PACKETS = LOST_PACKETS + var;
	}

	void IMU_Sensor::calibrate_imu(void)
	{
		float magx_off, magy_off, magz_off;
		//gyr
		gyr[3] = gyr[0];
		gyr[4] = gyr[1];
		gyr[5] = gyr[2];

		//magnet
		magx_off = mag[0] - hardiron[0];
		magy_off = mag[1] - hardiron[1];
		magz_off = -mag[2] - hardiron[2];

		mag[3] = comp[0][0] * magx_off + comp[0][1] * magy_off + comp[0][2] * magz_off;
		mag[4] = comp[1][0] * magx_off + comp[1][1] * magy_off + comp[1][2] * magz_off;
		mag[5] = comp[2][0] * magx_off + comp[2][1] * magy_off + comp[2][2] * magz_off;
	}

	void IMU_Sensor::sensor_fusion(void)
	{
		//pitch
		pitch[0] = ALPHA_ACC*rad2deg(atan2(-acc[1], -acc[0])) + ALPHA_GYRO*(pitch[0] + (-gyr[2]) * GAIN_GYRO * (1 / FREQ_GYRO));
		x_pitch[0] = pitch[0];
		pitch[1] = num_lp_10Hz[0] * x_pitch[0] + num_lp_10Hz[1] * x_pitch[1] - den_lp_10Hz[1] * pitch[2];
		x_pitch[1] = x_pitch[0];
		pitch[2] = pitch[1];
		pitchraw = pitch[2];
		pitchctrl = pitchraw - pitchoffset;

		//roll
		roll[0] = ALPHA_ACC*rad2deg(atan2(-acc[2], -acc[0])) + ALPHA_GYRO*(roll[0] + gyr[1] * GAIN_GYRO * (1 / FREQ_GYRO));
		x_roll[0] = roll[0];
		roll[1] = num_lp_10Hz[0] * x_roll[0] + num_lp_10Hz[1] * x_roll[1] - den_lp_10Hz[1] * roll[2];
		x_roll[1] = x_roll[0];
		roll[2] = roll[1];
		rollraw = roll[2];
		rollctrl = rollraw - rolloffset;

		//yaw
		float cP, sP, cR, sR;
		/*cP = cos(deg2rad(-pitch[2]));
		sP = sin(deg2rad(-pitch[2]));
		cR = cos(deg2rad(roll[2]));
		sR = sin(deg2rad(roll[2]));*/

		cP = cos(deg2rad(-pitch[1]));
		sP = sin(deg2rad(-pitch[1]));
		cR = cos(deg2rad(roll[1]));
		sR = sin(deg2rad(roll[1]));

		xh_cal = cP*cR*mag[3] - sP*mag[4] + cP*sR*mag[5];
		yh_cal = sP*cR*mag[3] + cP*mag[4] + sP*sR*mag[5];
		zh_cal = -sR*mag[3] + 0 * mag[4] + cR*mag[5];

		yaw[0] = ALPHA_MAG*rad2deg(atan2(-zh_cal, -yh_cal)) + (1 - ALPHA_MAG)*(yaw[0] + gyr[0] * GAIN_GYRO * (1 / FREQ_GYRO));
		x_yaw[0] = yaw[0];
		yaw[1] = num_lp_10Hz[0] * x_yaw[0] + num_lp_10Hz[1] * x_yaw[1] - den_lp_10Hz[1] * yaw[2];
		x_yaw[1] = x_yaw[0];
		yaw[2] = yaw[1];
		yawraw = yaw[0];//yaw[2];//

		acc_flat_yaw_tab[i_nb_acc] = absf(acc[0]);
		i_nb_acc++;
		if (i_nb_acc == nb_mean_acc)
		{
			mean_acc_flat_yaw = mean_tabf(acc_flat_yaw_tab, nb_mean_acc);
			volatile int i_acc_init;
			for (i_acc_init = 0; i_acc_init < nb_overlap_mean_acc; i_acc_init++)
				acc_flat_yaw_tab[i_acc_init] = acc_flat_yaw_tab[nb_mean_acc - (nb_overlap_mean_acc - i_acc_init)];
			i_nb_acc = nb_overlap_mean_acc;
		}

		if (IMU_type == HEADSET_IMU || IMU_type == IMU2 || IMU_type == IMU3)
		{
			//rotate yaw to zero
			float cPo, sPo, cRo, sRo, cYo, sYo;
			//cPo = cos(deg2rad(pitchoffset));  sPo = sin(deg2rad(pitchoffset));
			//cRo = cos(deg2rad(-rolloffset));  sRo = sin(deg2rad(-rolloffset)); 
			cPo = 1; sPo = 0; cRo = 1; sRo = 0;
			cYo = cos(deg2rad(-yawoffset));
			sYo = sin(deg2rad(-yawoffset));

			xh_ctrl = cPo*cRo			  *	xh_cal + sPo*cRo			  *	yh_cal - sRo	* zh_cal;
			yh_ctrl = -(sYo*cPo*sRo + cYo*sPo)*	xh_cal + (cYo*cPo - sYo*sPo*sRo)*	yh_cal - sYo*cRo* zh_cal;
			zh_ctrl = (cYo*cPo*sRo - sYo*sPo) *	xh_cal + (cYo*sPo*sRo + sYo*cPo)*	yh_cal + cYo*cRo* zh_cal;

			yaw_ctrl[0] = ALPHA_MAG_CTRL*rad2deg(atan2(-zh_ctrl, -yh_ctrl)) + ALPHA_GYRO_CTRL*(yaw_ctrl[0] + gyr[0] * GAIN_GYRO * (1 / FREQ_GYRO));
			x_yaw_ctrl[0] = yaw_ctrl[0];
			yaw_ctrl[1] = num_lp_10Hz[0] * x_yaw_ctrl[0] + num_lp_10Hz[1] * x_yaw_ctrl[1] - den_lp_10Hz[1] * yaw_ctrl[2];
			x_yaw_ctrl[1] = x_yaw_ctrl[0];
			yaw_ctrl[2] = yaw_ctrl[1];
			yawctrl = yaw_ctrl[2];
			//yawctrl = yawraw - yawoffset;

			if ((mean_acc_flat_yaw <= min_acc_calib) || (mean_acc_flat_yaw >= max_acc_calib))
				yawctrl = 0;

			if (CALIB_IMU_INDEX != -1 && CALIB_IMU_INDEX != -2)
			{
				float r_mean, p_mean;
				if (CALIB_TYPE == NEUTRAL) //|| CALIB_TYPE == HEADING_OFFSET)
				{
					pitch_calib[CALIB_IMU_INDEX] = pitchraw;
					roll_calib[CALIB_IMU_INDEX] = rollraw;
					yaw_calib[CALIB_IMU_INDEX] = yawraw;
					//acc_calib[CALIB_IMU_INDEX] = absf(acc[0]);
				}
				else if (CALIB_TYPE == HEADING_OFFSET)
				{
					acc_calib[CALIB_IMU_INDEX] = absf(acc[0]);
				}
				else
				{
					pitch_calib[CALIB_IMU_INDEX] = pitchctrl;
					roll_calib[CALIB_IMU_INDEX] = rollctrl;
					yaw_calib[CALIB_IMU_INDEX] = yawctrl;
				}
				CALIB_IMU_INDEX++;
				if (CALIB_IMU_INDEX >= NB_VALUES_CALIB_CURRENT)
				{
					CALIB_IMU_INDEX = -2;
					switch (CALIB_TYPE)
					{
					case NEUTRAL:
						pitchoffset		= mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						rolloffset		= mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						yawoffset		= mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						//CALIB_TYPE = 0;
						break;
					case HEADING_OFFSET:
						mean_acc_calib	= mean_tabf(acc_calib, NB_VALUES_CALIB_CURRENT);
						max_acc_calib	= max_tabf(acc_calib, NB_VALUES_CALIB_CURRENT);
						min_acc_calib	= min_tabf(acc_calib, NB_VALUES_CALIB_CURRENT);
						margin_acc_calib = (max_acc_calib - min_acc_calib) / 2;
						//CALIB_TYPE = 0;
						break;
					case MAX_FORTH:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_max_forw = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_forw = amp_max_forw*port_min_max;
						zone_forw = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_forw < 0)
							zone_forw += 360;
						yaw_ctrl_forw = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						//CALIB_TYPE = 0;
						break;
					case MAX_BACK:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_max_back = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_back = amp_max_back*port_min_max;//0.05;//
						zone_back = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_back < 0)
							zone_back += 360;
						yaw_ctrl_back = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						//CALIB_TYPE = 0;
						break;
					case MAX_RIGHT:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_max_right = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_right = amp_max_right*0.05;//port_min_max;
						zone_right = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_right < 0)
							zone_right += 360;
						yaw_ctrl_right = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						//CALIB_TYPE = 0;
						break;
					case MAX_LEFT:
						p_mean = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						r_mean = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_max_left = sqrt(p_mean*p_mean + r_mean*r_mean);
						amp_min_left = amp_max_left*port_min_max;
						zone_left = atan2(p_mean, r_mean) * 180 / M_PI;
						if (zone_left < 0)
							zone_left += 360;
						yaw_ctrl_left = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						//CALIB_TYPE = 0;
						break;
					case MAX_ROT_RIGHT:
						rot_max_right = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						rot_min_right = rot_max_right*(port_min_max + 0.2);
						pitchrotright = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						rollrotright = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_rot_right = sqrt(pitchrotright*pitchrotright + rollrotright*rollrotright);
						//CALIB_TYPE = 0;
						break;
					case MAX_ROT_LEFT:
						rot_max_left = mean_tabf(yaw_calib, NB_VALUES_CALIB_CURRENT);
						rot_min_left = rot_max_left*(port_min_max + 0.2);
						pitchrotleft = mean_tabf(pitch_calib, NB_VALUES_CALIB_CURRENT);
						rollrotleft = mean_tabf(roll_calib, NB_VALUES_CALIB_CURRENT);
						amp_rot_left = sqrt(pitchrotleft*pitchrotleft + rollrotleft*rollrotleft);
						//CALIB_TYPE = 0;
						break;
					//case HEADING_OFFSET:
						//yawrawmean = mean_tabf(yaw_calib, NB_VALUES_CALIB);
						//CALIB_TYPE = 0;
						//break;
					default:
						break;
					}
				}
			}
		}
		else if (IMU_type == REFERENCE_IMU)
		{
			if (index_gyr == nb_val_gyr)
				rotation_detected = detect_wheelchair_rotation();
			index_gyr = 0;
		}
	}

	char IMU_Sensor::detect_wheelchair_rotation()
	{
		//test rotation chaise
		MeanVelYaw = mean_tabf(moy_gyr, nb_val_gyr);
		char InhibiteZctrl = 0;
		if (MeanVelYaw >= ThRotReference)
			InhibiteZctrl = 1;
		return InhibiteZctrl;
	}

	float IMU_Sensor::max_tabf(float * tab, int taille)
	{
		int g; float max = tab[0];
		for (g = 0; g < taille; g++)
		{
			if (max < tab[g])
				max = tab[g];
		}
		return max;
	}

	float IMU_Sensor::min_tabf(float * tab, int taille)
	{
		int g; float min = tab[0];
		for (g = 0; g < taille; g++)
		{
			if (min > tab[g])
				min = tab[g];
		}
		return min;
	}

	float IMU_Sensor::mean_tabf(float * tab, int taille)
	{
		int g; float sum = 0.0;
		for (g = 0; g < taille; g++)
			sum = sum + tab[g];
		return (sum / taille);
	}

	float IMU_Sensor::C2_2(int var)
	{
		float var1, var2;
		int mask = 32767; //mask 0x7FFF
		var1 = var & mask;
		var2 = (var >> 15)*(-32768);
		return (var1 + var2);
	}

	float IMU_Sensor::absf(float x)
	{
		float y;
		if (x >= 0)
			y = x;
		else
			y = -x;
		return y;
	}

	float IMU_Sensor::rad2deg(float angle_rad)
	{
		return angle_rad *(180 / M_PI);
	}

	float IMU_Sensor::deg2rad(float angle_deg)
	{
		return angle_deg *(M_PI / 180);
	}
}
