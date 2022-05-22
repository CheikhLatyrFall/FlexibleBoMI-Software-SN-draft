#ifndef __CLASSIFIER_COLLECTOR_H_
#define __CLASSIFIER_COLLECTOR_H_

#include "PythonBridge.h"


namespace KinDrv{
	class ClassifierCollector {
		public :
			ClassifierCollector();
			~ClassifierCollector();
			PythonBridge myBridge; 
			PyObject *vector;
			int Close_bridge(void);
			int Classifier_class(int classifier_type);
			int Fill_vector(float* data, int lenght);
			int Clean_vector(void);
	};
}
#endif //__CLASSIFIER_COLLECTOR_H_
