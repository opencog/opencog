
#include "MindAgent.h"

namespace opencog {

class QueryProcessor : public MindAgent
{
	public:
		virtual void run(CogServer *server);

};
}

#include <stdio.h>

using namespace opencog;

void QueryProcessor::run(CogServer *server)
{
	printf ("hello world\n");

}

