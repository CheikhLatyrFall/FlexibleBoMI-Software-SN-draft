import numpy as np
import math
import struct
import array
import time
import wave
import os
import sys
import sklearn.svm
import tools
from sklearn.externals import joblib
from threading import Lock


class Predict(object):
	
	def __init__(self):
		self.__model=self._load_classifier("sentinel_nord_trained_svm")
		self.__lock=Lock()
		#self.__lda=self.__load_classsifier("sentinelle_nord_lda");
		
	def _load_classifier(self,svm_model):
		try : 
			model=joblib.load("SVM_Model/%s.pkl"%svm_model)
			print tools.bcolors.OKGREEN + "In predict - Classifier loaded %s - EXIT_SUCCESS"%svm_model + tools.bcolors.ENDC
			return model
		except IOError :
			print tools.bcolors.FAIL +"In preditc - ERROR - predictUnable to load the Classifier %s - EXIT_FAILURE"%svm_model+tools.bcolors.ENDC
			return
	
	def _predict_vector(self,vector):
		 self.__lock.acquire()
		 myvector=np.array(vector).reshape(1, -1)
		 R=-1
		 R=self.__model.predict(myvector)
		 print tools.bcolors.OKGREEN + "~~~~~~~~~~~ la classe est ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%s - ~~~~~~ "%R+tools.bcolors.ENDC
		 self.__lock.release()
		 return R
	
	def _predict__lda_vector(self,vector):
		 myvector=np.array(vector).reshape(1, -1)
		 R=-1
		 R=self.__lda.predict(myvector)
		 print R
		 return R
