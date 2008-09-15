/*
 * QueryProcessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_QUERY_PROCESSOR_H
#define _OPENCOG_QUERY_PROCESSOR_H

#include <opencog/atomspace/types.h>
#include <opencog/server/MindAgent.h>

namespace opencog {

class AtomSpace;

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

} // namespace opencog

#endif // _OPENCOG_QUERY_PROCESSOR_H
