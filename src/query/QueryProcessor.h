/*
 * QueryProcoessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "MindAgent.h"
#include "Node.h"

namespace opencog {

class QueryProcessor : public MindAgent
{
	private:
		Node *node;
		const char *match_name;
		bool match_node_name(Atom *);
		void do_assertion(Handle);

	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);

};
}

