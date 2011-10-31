#ifndef LAYER_FINISHED_CALLBACK_H
#define LAYER_FINISHED_CALLBACK_H

class LayerFinishedCallback {
	public:
		virtual ~LayerFinishedCallback(){}
		virtual void callback(int run, int layer) = 0; 
};

#endif