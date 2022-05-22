#include "classifiercollector.h"
#include <string>
#include <vector>


namespace KinDrv{
	enum{LDA,SVM};

	ClassifierCollector::ClassifierCollector(){
		vector = PyList_New(0);
	}

	ClassifierCollector::~ClassifierCollector(){
		
	}
	int ClassifierCollector::Close_bridge(){
		int status;
		status=myBridge.Close_bridge();
		return status;
	}

	int ClassifierCollector::Classifier_class(int classifier_type){
		int classification=-1;
		switch(classifier_type){
			case LDA :
				classification = myBridge.predict_lda(vector);
				break;
			case SVM :
				classification = myBridge.predict_svm(vector);
				break;
		}
		
		return classification;
	}

	int ClassifierCollector::Fill_vector(float* data, int length){
		PyObject *mylist = PyList_New(length);
		int i;
		for (i = 0; i<length; ++i) {
			PyList_SET_ITEM(mylist, i, PyFloat_FromDouble(data[i]));
		}
		vector = Py_BuildValue("(O)", mylist);
		/*
		for (i = 0; i <length; i++)
				{
					//PyList_Append(vector,Py_BuildValue("(f)", data[i]));
					
				}
		*/
		Py_DECREF(mylist);
		return EXIT_SUCCESS;
	}


	int ClassifierCollector::Clean_vector(){
		vector = PyList_New(0);
		return EXIT_SUCCESS;
	}
}
