#include "PythonBridge.h"

namespace KinDrv{
	/* The code below is from Ulysse Project */


	PythonBridge::PythonBridge(){
		pNameSVM=NULL;
		pModuleSVM=NULL;
		pDictSVM=NULL;
		pClassSVM=NULL;
		pInstanceSVM=NULL;
		const char *python_information[] = { "PythonFile", "prediction", "Predict" };
		//Initialize the Python Interpreter
		if (!Py_IsInitialized()){
			Py_Initialize();
		}
		//Build the name object
		PyRun_SimpleString("import sys; sys.path.insert(0,'/home/pi/Project/PythonFile')");
		pNameSVM = PyString_FromString(python_information[1]);
		if (pNameSVM==NULL){
			PyErr_Print();
			exit(EXIT_FAILURE);
		}
		//Load the module object
		pModuleSVM = PyImport_Import(pNameSVM);
		if (pModuleSVM==NULL){
			PyErr_Print();
			exit(EXIT_FAILURE);
		}
		//pDict is a borrowed reference
		pDictSVM = PyModule_GetDict(pModuleSVM);
		if (pDictSVM==NULL){
			PyErr_Print();
			exit(EXIT_FAILURE);
		}
		//Build the name of the callable class
		pClassSVM = PyDict_GetItemString(pDictSVM, python_information[2]);
		
		pInstanceSVM= NULL;
		//Create an instance of the class
		if (PyCallable_Check(pClassSVM))
		{
			pInstanceSVM = PyObject_CallObject(pClassSVM, NULL);
		}
		if (pInstanceSVM==NULL){
			PyErr_Print();
			exit(EXIT_FAILURE);
		}
	}

	PythonBridge::~PythonBridge(){
		
	}

	int PythonBridge::predict_svm(PyObject* vector){
		int classification = 0;
		PyObject *pValue;
		pValue = PyObject_CallMethod(pInstanceSVM,(char*)"_predict_vector",(char*)"O", vector);
		if (pValue != NULL)
		{
			classification = PyInt_AsLong(pValue);
			Py_DECREF(pValue);
			Py_DECREF(vector);
		}
		else
		{
			//PyErr_Print();
			exit(1);
		}
		return classification;
	}


	int PythonBridge::predict_lda(PyObject* vector){
		int classification = 0;
		PyObject *pValue;
		pValue = PyObject_CallMethod(pInstanceSVM,(char*)"_predict__lda_vector",(char*)"O", vector);
		if (pValue != NULL)
		{
			classification = PyInt_AsLong(pValue);
			Py_DECREF(pValue);
			Py_DECREF(vector);
		}
		else
		{
			PyErr_Print();
		}
		return classification;
	}

	int PythonBridge::Close_bridge(){
		Py_DECREF(pInstanceSVM);
		Py_DECREF(pDictSVM);
		Py_DECREF(pModuleSVM);
		Py_DECREF(pNameSVM);
		Py_Finalize();
		return EXIT_SUCCESS;
	}
}
