#ifndef __CLASSIFIER_MANAGER_H_
#define __CLASSIFIER_MANAGER_H_
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include "classifiercollector.h"
#include "ringbuffer.h"
#include "dataIO.h"

namespace KinDrv{
	class ClassifierManager{
		public :
			ClassifierManager(int length_vector_in, int length_vector_out);
			~ClassifierManager();
			RingBuffer *myBuffer;
			ClassifierCollector *myClassifier;
			dataIO *myData;
			void read_classify();
			void read_save(int label);
			void write_data();

		// array
			float* out;
			float* in;
		// classifier type;
			int classifier_type;
		
		// label
			int label_return;
		// available ressources 
			int available;
			int running;
			
		// semaphore 
			int     oflag;
			mode_t  mode;
			sem_t *Put_data_IN;
			sem_t *Put_data_WRITE;
			sem_t *Get_data_IN;
			sem_t *Get_data_READ;
		private :
		// length
			int len_out;
			int len_in;
			int status;
		
	};
}
#endif // __CLASSIFIER_MANAGER_H_
