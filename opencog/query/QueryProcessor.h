/*
 * QueryProcessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_QUERY_PROCESSOR_H
#define _OPENCOG_QUERY_PROCESSOR_H

#include <opencog/atomspace/types.h>
#include <opencog/server/Agent.h>
#include <opencog/server/Factory.h>
#include <opencog/server/Module.h>

namespace opencog
{

// forward declarations
class AtomSpace;

class QueryProcessor : public Agent, public Module
{

private:
	unsigned int cnt;
	AtomSpace *atom_space;
	bool do_assertion(Handle);

	Handle completion_handle;
	bool check_done(Handle);

public:

	static Factory<QueryProcessor, Agent> factory;

	virtual const ClassInfo& classinfo() const { return info(); }
	static const ClassInfo& info() {
		static const ClassInfo _ci("opencog::QueryProcessor");
		return _ci;
	}

	QueryProcessor(void);
	virtual ~QueryProcessor();
	virtual void run(CogServer *server);

};

} // namespace opencog

#endif // _OPENCOG_QUERY_PROCESSOR_H
