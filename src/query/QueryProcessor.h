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

		Node *node; // for match_node_name only
		const char *match_name; // for match_node_name only
		bool match_node_name(Atom *);

		std::vector<Handle> varlist;
		bool check_for_query(Atom *);

		AtomSpace *atom_space;
		bool do_assertion(Handle);

	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);

};
}

