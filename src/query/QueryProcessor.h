
#include "MindAgent.h"

namespace opencog {

class QueryProcessor : public MindAgent
{
	private:
		void do_assertion(Handle);
		bool is_qvar(Handle);

	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);

};
}

