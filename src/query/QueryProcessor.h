
#include "MindAgent.h"

namespace opencog {

class QueryProcessor : public MindAgent
{
	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);

};
}

