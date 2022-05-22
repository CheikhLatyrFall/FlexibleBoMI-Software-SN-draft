#include "classifier_manager.h"
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <sys/stat.h>
namespace KinDrv{
	ClassifierManager::ClassifierManager(int length_vector_in, int length_vector_out){
			myBuffer= new RingBuffer(length_vector_out,length_vector_in);
			myClassifier= new ClassifierCollector();
			myData= new dataIO();
			out=new float[length_vector_out];
			in=new float[length_vector_in];
			len_out=length_vector_out;
			len_in=length_vector_in;
			label_return=-1;
			classifier_type=1;
			available=0;
			running=0;
			status=0;
			// semaphore
			oflag = O_CREAT;
			mode = S_IRUSR | S_IWUSR;
			Put_data_IN=sem_open("pPut_data_IN",oflag, mode,1);
			if (Put_data_IN == (void *)-1) {
				perror("failed");
			}
			Put_data_WRITE=sem_open("pPut_data_IN",oflag, mode,0);
			if (Put_data_WRITE == (void *)-1) {
				perror("failed");
			}
			Get_data_IN=sem_open("pGet_data_IN",oflag, mode,1);
			if (Get_data_IN == (void *)-1) {
				perror("failed");
			}
			Get_data_READ=sem_open("pGet_data_READ",oflag, mode,0);
			if (Get_data_READ == (void *)-1) {
				perror("failed");
			}
	}

	ClassifierManager::~ClassifierManager(){
		delete [] out;
		delete [] in;
		delete myBuffer;
		delete myClassifier;
		delete myData;
		sem_close(Put_data_IN);
		sem_close(Put_data_WRITE);
		sem_close(Get_data_IN);
		sem_close(Get_data_READ);
		sem_unlink("pPut_data_IN");
		sem_unlink("pPut_data_IN");
		sem_unlink("pGet_data_IN");
		sem_unlink("pGet_data_READ");
	}
	
	void ClassifierManager::read_classify(){
			status=myBuffer->read(out);
			if(status==1){
				myClassifier->Fill_vector(out,len_out);
				label_return=myClassifier->Classifier_class(classifier_type);
				available=1;
				printf("class is :%i\n",label_return);
				available=0;
				myClassifier->Clean_vector();
			}
	}
	
	void ClassifierManager::read_save(int label){
		status=myBuffer->read(out);
			if(status==1){
				printf("#############################SAVE DATA ###########\n");
				myData->saveDatabase(label,out,1080,"Database2/");
			}
	}
	
	void ClassifierManager::write_data(){
				//sem_trywait(Put_data_WRITE);
				myBuffer->write(in);
				//sem_post(Put_data_IN);
	}
}



