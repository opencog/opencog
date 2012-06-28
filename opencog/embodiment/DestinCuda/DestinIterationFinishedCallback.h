#ifndef DestinIterationFinishedCallback_H
#define DestinIterationFinishedCallback_H

#include "DestinKernel.h"

#include "INetwork.h"

struct RunningInfo {
	int sequence_step;
	int image_count;
	int image_label;
	int layer;
};

class DestinIterationFinishedCallback {
	public:
		virtual ~DestinIterationFinishedCallback(){}
		virtual void callback(RunningInfo & info, INetwork & network) = 0;
};

#endif
