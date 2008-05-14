/*
 * QueryProcoessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "MindAgent.h"

namespace opencog {

class QueryProcessor : public MindAgent
{
	private:
		AtomSpace *atom_space;
		bool do_assertion(Handle);

	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);
};
}

