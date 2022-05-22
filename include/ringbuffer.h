#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

/**
	@author Quentin MASCRET <quentin.mascret.1@ulaval.ca>
	@version 1.0-dev
	#data 2017-11-17
*/
#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <sys/stat.h>

#define MAX_SIZE 4320


namespace KinDrv{
	class RingBuffer {
		public :
			RingBuffer(int WindowSize, int DataSize);
			~RingBuffer();
			int tail;
			int head;
			int max_size;
			int flag;
			float Buffer[MAX_SIZE];
			int window_size;
			int data_size;
		
			int     oflag;
			mode_t  mode;
			
			// proto
			void write(float *data);
			int IsFull(float *data);
			int read(float *out);
			int IsEmpty(float *out);
			int FlushRingBuffer();
		private :
			// semaphore 
			sem_t *ReadCountAccess;
			sem_t *ServiceQueue;
			sem_t *RessourceAccess;
			int ReadCount;
			
			
			
	};
}
#endif //__RING_BUFFER_H__
