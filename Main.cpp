#define _USE_MATH_DEFINES
#define _USE_DISLIN
#define _SET_SERVER

#include "User_Interface.h"

using namespace Control_IMU_JACO;
using namespace KinDrv;
using namespace std;

int main(int argc, char*argv[])
{
	User_Interface User;
	User.initialize();
	User.set();
		
	while (1)
	{
		User.routine();
	}
	
	return 0;
}





