import numpy as np
import math
import struct
import array
import time
import wave
import os
import sys
import sklearn.svm
import sklearn.decomposition
import sklearn.ensemble
from sklearn.svm import SVC
from sklearn.model_selection import GridSearchCV
from sklearn.model_selection import train_test_split
from sklearn.model_selection import StratifiedShuffleSplit
from sklearn.model_selection import StratifiedKFold
import tools
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from sklearn.metrics import confusion_matrix
import pprint as pp
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import RobustScaler
from sklearn.externals import joblib
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.discriminant_analysis import QuadraticDiscriminantAnalysis
import scipy.io as sio
__author__="Quentin MASCRET <quentin.mascret.1@ulaval.ca>"
__date__="2017-11-19"
__version__="1.0-dev"

class MidpointNormalize(Normalize):

    def __init__(self, vmin=None, vmax=None, midpoint=None, clip=False):
        self.midpoint = midpoint
        Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        x, y = [self.vmin, self.midpoint, self.vmax], [0, 0.5, 1]
        return np.ma.masked_array(np.interp(value, x, y))
     

class MachineLearning(object):
	
	def __init__(self,length):
		self.__path="Database/"
		self.__length=length
		self.__features=np.array([]).reshape(self.__length,0)
		self.__class=np.array([]).reshape(1,0)
		self.__C_range=np.logspace(0,10,15)
		self.__gamma_range=np.logspace(-8,1,15)
		self.__params=dict(C=self.__C_range, gamma=self.__gamma_range)
		pass

	def _read_features(self):
		
		os.chdir(self.__path)
		listdirectory = os.listdir(".")
		for filename in listdirectory :
			if self._class_dictionnary(filename)>0 : 
				os.chdir(filename)
				listoffile=os.listdir(".")
				file_error=0
				for allfile in listoffile :
					if(allfile ==".DS_Store"):
						file_error+=1
					if(allfile !=".DS_Store"):
						data=(np.loadtxt(allfile)).reshape(self.__length,1)
						self.__features=np.hstack([self.__features,data])
				classN=np.matlib.repmat(self._class_dictionnary(filename),1,len(listoffile)-file_error)
				self.__class=np.hstack([self.__class,classN])
				os.chdir("../")
		os.chdir("../")
		np.savetxt("t.txt", self.__features.T)
		print self.__class
		print tools.bcolors.OKGREEN + "In machine learning - __read_features : EXIT_SUCCESS" +tools.bcolors.ENDC
		return self.__features.T, self.__class[0]
	
	def _read_features_from_xls(self):
		os.chdir(self.__path)
		mat_contents=sio.loadmat('test_training_latyr.mat')
		os.chdir("../")
		classes=mat_contents['Etiquette2']
		accx=mat_contents['ACCX']
		accy=mat_contents['ACCY']
		accz=mat_contents['ACCZ']
		gyrox=mat_contents['GYROX']
		gyroy=mat_contents['GYROY']
		gyroz=mat_contents['GYROZ']
		magx=mat_contents['MAGX']
		magy=mat_contents['MAGY']
		magz=mat_contents['MAGZ']
		pitch=mat_contents['PitchCtrl']
		roll=mat_contents['RollCtrl']
		yaw=mat_contents['YawCtrl']
		features=np.hstack([accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz,pitch,roll,yaw])
		return features, classes.ravel()
		
	def _class_dictionnary(self,filename):
		return { 
        'random' 	: 1,
        'assis' 	: 2,
        'debout' 	: 3,
        'marche' 	: 4,
        'cours'		: 5, 
        }.get(filename,0) # zero is default class	
	
	def _train_rbf_svm(self,features,classes):
		# scaler
		scaler = RobustScaler()
		features = scaler.fit_transform(features)
		
		#kfold
		cv = StratifiedShuffleSplit(n_splits=10, test_size=0.5, train_size=0.5,random_state=4)
		cv.get_n_splits(features,classes)
		print tools.bcolors.OKBLUE + "In machine learning - Info - Running ..." + tools.bcolors.ENDC	
		grid =GridSearchCV((SVC(kernel="rbf")),param_grid=self.__params,cv=cv,verbose=20,n_jobs=10) 
		grid.fit(features,classes)
		print("The best parameters are %s with a score of %0.2f"% (grid.best_params_, grid.best_score_))
		
		scores = [x[1] for x in grid.cv_results_]
		scores=[entry for entry in grid.cv_results_["mean_test_score"]] #grid_scores_
		# scores=[entry.mean_validation_score for entry in grid.grid_scores_]
		scores = np.array(scores).reshape(len(self.__C_range), len(self.__gamma_range))
		#scores = grid.grid_scores_.mean_validation_score#['mean_validation_score']#.reshape(len(C),len(Gamma))
		
		plt.figure(figsize=(8, 6))
		plt.subplots_adjust(left=.2, right=0.95, bottom=0.15, top=0.95)
		plt.imshow(scores, interpolation='nearest', cmap=plt.cm.hot, norm=MidpointNormalize(vmin=0.2, midpoint=0.92))
		plt.xlabel('gamma')
		plt.ylabel('C')
		plt.colorbar()
		plt.xticks(np.arange(len(self.__gamma_range)), self.__gamma_range, rotation=45)
		plt.yticks(np.arange(len(self.__C_range)), self.__C_range)
		plt.title('Validation accuracy')
		plt.show()
		return grid
	
	def _train_lda_classifier(self,features,classes):
		lda_model=LinearDiscriminantAnalysis(solver="svd", store_covariance=True)
		lda_model.fit(features,classes)
		print lda_model.score(features,classes)
		return lda_model
	
	def _save_svm(self,model_name,svm_classifier):
		joblib.dump(svm_classifier,"SVM_Model/%s.pkl"%model_name)
		print tools.bcolors.OKGREEN + "In machine learning - __save : Model saved - EXIT_SUCCESS" + tools.bcolors.ENDC
   
	
	def _train_best_svm(self,params, features,classes):
		svm=(SVC(kernel="rbf",**params))
		svm.fit(features,classes)
		print tools.bcolors.HEADER + "In machine learning - __train_best_svm - SVM trained - EXIT_SUCESS" + tools.bcolors.ENDC
		return svm

		
	def _train(self):
		features,classes=self._read_features_from_xls()
		if len(features)==0 :
			print tools.bcolors.FAIL+ "In machine learning - __train - EXIT_FAILURE : No features "+tools.bcolors.ENDC
			return
		SVM_Model=self._train_rbf_svm(features,classes)
		self._save_svm("sentinel_nord_svm",SVM_Model)
		params=dict(gamma=SVM_Model.best_params_["gamma"], C=SVM_Model.best_params_["C"])
		SVM_trained=self._train_best_svm(params,features,classes)
		self._save_svm("sentinel_nord_trained_svm",SVM_trained)
		print tools.bcolors.OKGREEN + "In machine learning - TRAINNING PHASE - EXIT SUCCESS" + tools.bcolors.ENDC
	
	def _train_lda(self):
		features,classes=self._read_features_from_xls()
		if len(features)==0 :
			print tools.bcolors.FAIL+ "In machine learning - __train - EXIT_FAILURE : No features "+tools.bcolors.ENDC
			return
		lda_model=self._train_lda_classifier(features,classes)
		self._save_lda("sentinelle_nord_lda",lda_model)
		print tools.bcolors.OKGREEN + "In machine learning - TRAINNING PHASE - EXIT SUCCESS" + tools.bcolors.ENDC
	
	def _save_lda(self,model_name,lda_classifier):
		joblib.dump(lda_classifier,"LDA_Model/%s.pkl"%model_name)
		print tools.bcolors.OKGREEN + "In machine learning - __save : Model saved - EXIT_SUCCESS" + tools.bcolors.ENDC
  
if __name__=='__main__' :
	myTrainning=MachineLearning(1200);
	#myTrainning._train_lda()
	myTrainning._train()
