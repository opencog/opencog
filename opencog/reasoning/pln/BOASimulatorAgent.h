#ifndef _BOASIMULATORAGENT_H_
#define _BOASIMULATORAGENT_H_

#include "Singleton.h"
#include "MindAgent.h"
#include <util/PTLutils.h>
#include "tree.hh"

/**
	BOASimulatorAgent contains code to run a simple test with learning module code,
	using simple_evaluator as the fitness function. Currently this code is disabled
	due to a compilation problem with new learning module code (and this was based
	on BOA/MOSES code from March 2006).
*/

class BOASimulatorAgent : public MindAgent, public Singleton<BOASimulatorAgent>
{
protected:
	friend class Singleton<BOASimulatorAgent>;
	BOASimulatorAgent();
public:
	virtual void execute();	
};

void raw_print(tree<Vertex>& t, tree<Vertex>::iterator top, int level=0);

#endif //_BOASIMULATORAGENT_H_
