Assistive HMI Project
Author : Cheikh Latyr Fall
email  : cheikh-latyr.fall.1@ulaval.ca

The content of this project is confidential and is realised in the context of an RDC Project with Kinova Robotics.

Features :

- EMG 16 and 24 bits operation
- EMG in scale range
- Multisensor test


KEYPAD MAPPING:

Key			Environnement		Function							Note
D			Linux/Windows		START/STOP API
U				-				Lock/Unlock Keypad
SPACE			-				Enable/Disable Control
O				-				Change Mode - Button1
P				-				Change Mode - Button0				if _FINGER_CONTROL and _KINOVA_DEMO not defined
F				-				Control Trigger - Click 			if CONTROL_TRIGGER == SINGLE_CLICK
Z				-				Calibrate EMG Zero
E				-				Bring back arm to home (only)
V 				-				Validate Heading Offset				if WHEELCHAIR_IMU (CTRL_IMU.ReferenceActive) is here
W				-				Enable/Disable Yaw Angle Control
Q				-				Quit apllication
0 (num)			-				Calibrate IMU Zero
up arrow		-				Calibrate Max Forward
down arrow		-				Calibrate Max Backward
right arrow		-				Calibrate Max Right
left arrow		- 				Calibrate Max Left
6 (num)			-				Calibrate Max Rotation Right
4 (num)			-				Calibrate Max Rotation Left
7 (num)			-				Set Average Rot Variations on ACC

