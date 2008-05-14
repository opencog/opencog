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
		unsigned int cnt;
		AtomSpace *atom_space;
		bool do_assertion(Handle);

		Handle completion_handle;
		bool check_done(Handle);

	public:
		QueryProcessor(void);
		virtual ~QueryProcessor();
		virtual void run(CogServer *server);
};
}

