#include "PLN.h"

#ifndef USE_PSEUDOCORE

#ifndef AUTO_H_PTLEVALUATOR_H
  #define AUTO_H_PTLEVALUATOR_H
  #include "PTLEvaluator.h"
#endif

#ifndef AUTO_H_HANDLEENTRY_H
  #define AUTO_H_HANDLEENTRY_H
  #include "HandleEntry.h"
#endif

// Welter's comment: This seem not used anymore
//#ifndef AUTO_H_INFERENCEMINDAGENT_H
//  #define AUTO_H_INFERENCEMINDAGENT_H
//  #include "InferenceMindAgent.h"
//#endif

#ifndef AUTO_H_RULES_H
  #define AUTO_H_RULES_H
  #include "PLNRules/Rules.h"
#endif

#ifndef AUTO_H_BACKWARDINFERENCETASK_H
  #define AUTO_H_BACKWARDINFERENCETASK_H
  #include "BackwardInferenceTask.h"
#endif


namespace reasoning
{

BackwardInferenceTask::BackwardInferenceTask(const InferenceTaskParameters& _pars)
: pars(_pars)
{
	puts("BackwardInferenceTask created.");
}

BackwardInferenceTask::~BackwardInferenceTask()
{
}

void BackwardInferenceTask::cycle()
{
	puts("BackwardInferenceTask cycle()");	

/*HandleEntry* ret = PTLEvaluator::evaluate(pars);
	
	if (ret && ret->handle)
		printTree( ret->handle );	
	*/	
	finished = true;
}

void BackwardInferenceTask::finish()
{
	puts("BackwardInferenceTask finish()");
}

bool BackwardInferenceTask::ok() { return finished; }

} //reasoning

#endif //#ifndef USE_PSEUDOCORE
