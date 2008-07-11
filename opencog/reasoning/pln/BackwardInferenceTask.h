#ifndef _BACKWARDINFERENCETASK_H_
#define _BACKWARDINFERENCETASK_H_

#ifndef USE_PSEUDOCORE

namespace reasoning
{
	
class InferenceTaskParameters;

/**
	The goal/BackwardInferenceAgent should be replaced by this kind of a structure,
	which is not yet implemented beyond this simple skeleton.
*/

class BackwardInferenceTask
{
	const InferenceTaskParameters& pars;
	bool finished;
public:
	BackwardInferenceTask(const InferenceTaskParameters& _pars);

	virtual ~BackwardInferenceTask();
	
	virtual void cycle();

	/**
	 * Task Interface: This method is called when the Task has spent the amount of 
	 * resources it was granted by its EffortMonitor.
	 */ 	
	virtual void finish();
	bool ok();
};

}

#endif //#ifndef USE_PSEUDOCORE

#endif //_BACKWARDINFERENCETASK_H_
