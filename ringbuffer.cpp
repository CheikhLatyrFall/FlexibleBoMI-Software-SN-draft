/**
	@author Quentin MASCRET <quentin.mascret.1@ulaval.ca>
	@version 1.0-dev
	#data 2017-11-17
*/

#include "ringbuffer.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <sys/stat.h>

namespace KinDrv{
	RingBuffer::RingBuffer(int WindowSize, int DataSize){
		tail=0;
		head=0;
		max_size=MAX_SIZE;
		flag=0;
		int i;
		for (i=0;i<MAX_SIZE;i++){
			Buffer[i]=0.0;
		}
		window_size=WindowSize;
		data_size=DataSize;
		
		oflag = O_CREAT;
		mode = S_IRUSR | S_IWUSR;
		RessourceAccess=sem_open("pRessourceAccess",oflag, mode,1); //S_IRUSR | S_IWUSR
		if (RessourceAccess == (void *)-1) {
			perror("failed");
		}
		ReadCountAccess=sem_open("pReadCountAccess",oflag, mode,1);
		if (ReadCountAccess == (void *)-1) {
			perror("failed");
		}
		ServiceQueue=sem_open("pServiceQueue",oflag, mode,1);
		if (ServiceQueue == (void *)-1) {
			perror("failed");
		}
		ReadCount=0;
	}

	RingBuffer::~RingBuffer(){
		sem_close (RessourceAccess);
		sem_close (ReadCountAccess);
		sem_close (ServiceQueue);
		sem_unlink ("pRessourceAccess"); 
		sem_unlink ("pReadCountAccess"); 
		sem_unlink ("pServiceQueue"); 
	}

	int RingBuffer::IsFull(float *data){
		// local parameters
		int Next=head+data_size;
		int i,j;
		if (Next>=max_size){
			Next=0;
			flag=1;
		}
		
		if(Next==tail){
			return -1;
		}
		int index=head+data_size;
		j=0;
		for(i=head;i<index;i++){
			Buffer[i]=0;
			Buffer[i]=data[j];
			if (j>=data_size){
				return -1;
			}
			j++;
		}
		
		head=Next;
		//printf("Data write : %i ---->\n",head);
		return 0;
		
	}

	int RingBuffer::IsEmpty(float *out){
		
		// some parameters 
		int index;
		int Next;
		int i,j;
		int overflow=0;
		// 
		if (head== tail){
			return -1;
		}
		Next=tail+window_size;
		
		if ( Next >=max_size){
				Next = Next-max_size;
				flag=0;
				overflow=1;
		}
		if (Next >= head and flag ==0){
				return -1;
		}
		if (Next <= head and flag ==1){
				return -1;
		}
		if (overflow==1 && tail<=head){
			return -1;
		}
		
		index=tail+window_size;
		if(index>=max_size){
			j=0;
			overflow=0;
			for (i=tail;i<max_size;i++){
				out[j]=0;
				out[j]=Buffer[i];
				j++;
			}
			for (i=0;i<(window_size-(max_size-tail));i++){
				out[j]=0;
				out[j]=Buffer[i];
				if (j>=window_size){
					return -1;
				}
				j++;
			}
		}
		
		else {
			j=0;
			for (i=tail;i<Next;i++){
				out[j]=0;
				out[j]=Buffer[i];
				
				if (j>=window_size){
					return -1;
				}
				j++;
			}
		}
		
		tail=Next;
		//printf("Data read : %i\n",tail);
		return 0;

	}


	void RingBuffer::write(float *data){
		sem_trywait(ServiceQueue);
		sem_trywait(RessourceAccess);
		sem_post(ServiceQueue);
		IsFull(data);
		sem_post(RessourceAccess);
	}


	int RingBuffer::read(float *out){
		int status=0;
		sem_trywait(ServiceQueue);
		sem_trywait(ReadCountAccess);
		if(ReadCount==0){
			sem_wait(RessourceAccess);
		}
		ReadCount++;
		sem_post(ServiceQueue);
		sem_post(ReadCountAccess);
		status=IsEmpty(out);
		sem_trywait(ReadCountAccess);
		ReadCount--;
		if(ReadCount==0){
			sem_post(RessourceAccess);
		}
		sem_post(ReadCountAccess);
		if(status==0){
			return 1;
		}
		else{
			return 0;
		}
		 
	}
	
	int RingBuffer::FlushRingBuffer(){
		tail=0;
		head=0;
		max_size=MAX_SIZE;
		flag=0;
		int i;
		for (i=0;i<MAX_SIZE;i++){
			Buffer[i]=0.0;
		}
		printf("buffer clean and initialize \n");
		return EXIT_SUCCESS;
	}
}
