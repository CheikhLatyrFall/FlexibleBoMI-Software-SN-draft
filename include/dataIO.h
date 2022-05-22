#ifndef __DATA_IO_H__
#define __DATA_IO_H__

/**
	@author Quentin MASCRET <quentin.mascret.1@ulaval.ca>
	@version 1.0-dev
	#data 2017-11-17
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define NO_ERROR 0
#define INTERNAL_ERROR -1  

using namespace std;

namespace KinDrv{
	class dataIO {
		public : 
			dataIO();
			~dataIO();
			
			int saveDatabase(int currentClass, float *data,int length,const char* path);
			const char* GetClassLabel(int currentClass);
		protected :
			struct stat info;
	};
}
#endif // __DATA_IO_H__
