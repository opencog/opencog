/**
 * QueryProcessor.cc
 *
 * Process user queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "AtomSpace.h"
#include "CogServer.h"
#include "MindAgent.h"
#include "QueryProcessor.h"

using namespace opencog;

QueryProcessor::QueryProcessor(void)
{
}

QueryProcessor::~QueryProcessor()
{
}

void QueryProcessor::run(CogServer *server)
{
	AtomSpace *as = server->getAtomSpace();
	// Handle h = as->getHandle(qtype, "test");
	// Atom *atom = TLB::getAtom(h);
	
	// Look for recently asserted assertions.
	Type atype = ClassServer::getType("AssertionLink");
	std::list<Handle> asrt_list;
	as->getHandleSet(back_inserter(asrt_list), atype, NULL);

	// Loop over all recent assertions, and take care of them.
	while(!asrt_list.empty())
	{
		Handle h = asrt_list.front();
		do_assertion(h);
		asrt_list.pop_front();
		if (h) as->removeAtom(h);
	}

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
}

void QueryProcessor::do_assertion(Handle h)
{
	printf ("found handle=%p\n", h);
}
