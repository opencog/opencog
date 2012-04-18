#ifndef LAYER_FINISHED_CALLBACK_H
#define LAYER_FINISHED_CALLBACK_H

#include "DestinKernel.h"
struct RunningInfo {
	int sequence_step;
	int image_count;
	int image_label;
	int layer;
};

class LayerFinishedCallback {
	public:
		virtual ~LayerFinishedCallback(){}
		virtual void callback(RunningInfo & info, DestinKernel & layer ) = 0; 
};

#endif
