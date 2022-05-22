#include <python2.7/Python.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
namespace KinDrv{
	class PythonBridge {
		public :
			PythonBridge();
			~PythonBridge();
			PyObject *pNameSVM, *pModuleSVM, *pDictSVM, *pClassSVM, *pInstanceSVM;
			int predict_svm(PyObject* vector);
			int predict_lda(PyObject* vector);
			int Close_bridge();
	};
}
