#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Defined_Macro.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"

#ifdef _WIN32
#include "KinovaTypes.h"
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#endif

#ifndef _WIN32
#include "types.h"
#include <libkindrv/kindrv.h>
#endif

#ifndef _WIN32
using namespace KinDrv;
#endif

namespace Control_IMU_JACO
{
	
	class Control_HMI
	{
	public:

		IMU_Sensor HEAD_IMU;
		IMU_Sensor CHAIR_IMU;
		IMU_Sensor IMU3_IMU;
		EMG_Sensor EMG_1;
		EMG_Sensor EMG_2;
		EMG_Sensor EMG_3;
		EMG_Sensor EMG_4;
		int HEAD_IMU_STATE;
		int CHAIR_IMU_STATE;
		int IMU3_IMU_STATE;
		int EMG_1_STATE;
		int EMG_2_STATE;
		int EMG_3_STATE;
		int EMG_4_STATE;
		int limit_missing;

		char WBSN_MODE; //RECORDING or CONTROL

		//DIRECTION - COMMANDES
		float * DIRECTION_ETIQUETTE = NULL;
		float * RANDOM_ETIQUETTE = NULL;
		float DIRECTION;
		float CmdX_new; float CmdY_new;
		float CmdXout;	float CmdYout;
		float CmdZ; float CmdZ_new;
		float RightLeftComm; float ForBackComm; float UpDownComm;
		float RightLeftComm_ctrl; float ForBackComm_ctrl; float UpDownComm_ctrl;
		char B_INDEX, B_INDEX1, B_INDEX2;

		//Data 2D
		float x;									//roll + offset
		float y;									//pitch + offset
		float x0;									//roll brute
		float y0;									//pitch brute	
		float xoff;									//offset - position neutre roll
		float yoff;									//offset - position neutre pitch
		float Amp;
		float Theta;
		float Orientation;

		//Data 2D Pitch/Yaw
		float z_2;
		float Amp_2;
		float Theta_2;

		// Z
		float z0, z0mean;
		float z_ctrl; int cpt_neutral;
		float zoff;
		float z0_ref;
		int InhibiteZctrl;

		//CALIBRATION**************************************************************************************/
		int ZActive;
		int ReferenceActive;						//indique l'acitivation du noeud de référence
		int Valid;									//indique la validité des marges et zones définies
		int DiagoActive;							//0 = diagonale désactivée, 1 = diagonale activée
		float Theta_offset;							//0 .. 360 degrés, mapping
		float ThRotReference;						//vitesse de rotation seuil maximale du fauteuil

		bool BACK;
		float Amin_BACK;
		float Amax_BACK;
		float Amin_D;
		float Amax_D;
		float Amax;								//inclinaison maximale
		float Amin;									//seuil d'inclinaison

		float ZoneForw, DeltaForw;	//Angle Forward	, Marge Forwad
		float ZoneRight, DeltaRight;	//Angle Right	, Marge Right
		float ZoneLeft, DeltaLeft;	//Angle Left	, Marge Left
		float ZoneBack, DeltaBack;	//Angle Backward, Marge Backward
		float Ri1, Ri2;

		float AmaxForw, AmaxBack;
		float AminForw, AminBack;
		float AmaxRight, AmaxLeft;
		float AminRight, AminLeft;
		
		float RotmaxRight, RotmaxLeft;
		float RotminRight, RotminLeft;
		
		float YawCtrlForw, YawCtrlBack;
		float YawCtrlRight, YawCtrlLeft;

		
		float PitchRotRight, RollRotRight;
		float PitchRotLeft, RollRotLeft;
		float AmpRotRight, AmpRotLeft;

		int MODE_JACO, MODE_JACO_RETURNED;

		float DeltaRot;

		bool IMU_CALIBRATED, EMG1_CALIBRATED, EMG2_CALIBRATED, EMG3_CALIBRATED, EMG4_CALIBRATED;
		char imu_calibration[7];
		/**************************************************************************************************/

		Control_HMI();
		~Control_HMI();
		void initialize(char emg_precision);
		void set_default_calibration(void);
		void process_payload(int *PCKT, char mapping_mode);
		int update_calibration(void);
		void algo(char mapping_mode);
		float fmaxf(float x, float y);
#ifdef _WIN32
		int ReadJACOMode(void);
		JoystickCommand CommandJACO(int b);
		JoystickCommand PushButton(char buttonindex);
		JoystickCommand ReleaseButton(char buttonindex);
#else		
		jaco_joystick_t CommandJACO_RPI(int b);
#endif
	};
}
