
/**
	@author Quentin MASCRET <quentin.mascret.1@ulaval.ca>
	@version 1.0-dev
	#data 2017-11-17
*/

#include "dataIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/dir.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <fstream>
struct dirent *directory;
namespace KinDrv{
	dataIO::dataIO(){
		
	}

	dataIO::~dataIO(){
		
	};

	int dataIO::saveDatabase(int currentClass,float *data,int length,const char* folder_path){
		// parameters
		DIR *dirp;
		int nbOfFile=0;
		int error=EXIT_SUCCESS;
		int dir_err=0;
		int count;
		
		char *path= new char [strlen(folder_path)+
								strlen(GetClassLabel(currentClass))+1];
		//char* path= new char [strlen(folder_path)+
		//						strlen(GetClassLabel(currentClass))+1];
		
		// check if folder path exists 
		if( stat( folder_path, &info ) != 0 ){
			dir_err = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			if (dir_err==-1)
			{
				printf("Error during creating directory %s\n", folder_path);
				error=EXIT_FAILURE;
			}
		}
		strcpy(path,folder_path);
		strcat(path,GetClassLabel(currentClass)); 
		//strcat(p,GetClassLabel(currentClass));
		//strcpy(path,p);
		// check if Class label exists
		if( stat( path, &info ) != 0 ){
			dir_err = mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			if (dir_err==-1)
			{
				fprintf(stderr,"\n dataIO ERROR :unable to create current directory %s \n",path);
				exit(EXIT_FAILURE);
			}
		}

		// go in Class label folder
		dir_err=chdir(path);
		if (dir_err==-1)
		{
			fprintf(stderr,"\n dataIO ERROR :change working directory impossible \n");
			exit(EXIT_FAILURE);
		}
		
		// count file in the folder
		dirp = opendir(".");
		if (dirp)
		{
			while ((directory = readdir(dirp)) != NULL)
			{
				if (strcmp(directory->d_name, ".") != 0 &&
							strcmp(directory->d_name, "..") != 0 &&
							strcmp(directory->d_name, ".DS_Store") != 0){
					nbOfFile++;
				}
			}

			closedir(dirp);
		}
		
		// create filename
		char* name = new char [strlen(GetClassLabel(currentClass))+1];
		strcpy(name,GetClassLabel(currentClass)); 
		char filename[100];
		sprintf(filename, "%s_%i.txt",name, nbOfFile);
		
		
		// open file
		ofstream myfile (filename);
		if (myfile.is_open())
		{
			for(count = 0; count < length; count ++){
				myfile << data[count] << "\n" ;
				
			}
		myfile.close();
		}
		else {
			fprintf(stderr,"\ndataIO ERROR : unable to open current file %s\n",filename);
			error=INTERNAL_ERROR;
		}
		dir_err=chdir("../../");
		free(name);
		free(path);
		return error; 
	}

	const char*  dataIO::GetClassLabel(int currentClass){
		switch(currentClass){
			case -1:
				return "empty";
			case 0:
				return "random";
			case 1:
				return "sit_to_stand";
			case 2 :
				return "walk";
			case 3 :
				return "stairs";
			case 4 :
				return "standing";
			case 5 :
				return "seated";
			case 6 :
				return "lying";
			default :
				fprintf(stderr,"\ndataIO ERROR : Unknown Class label\n");
				exit(EXIT_FAILURE);
		}
	}
}
