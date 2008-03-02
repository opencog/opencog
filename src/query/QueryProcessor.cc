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
#include "Foreach.h"
#include "MindAgent.h"
#include "QueryProcessor.h"
#include "RelexQuery.h"

using namespace opencog;

// ----------------------------------------
QueryProcessor::QueryProcessor(void)
{
}

QueryProcessor::~QueryProcessor()
{
	atom_space = NULL;
}

void QueryProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();
	// Handle h = as->getHandle(qtype, "test");
	
	// Look for recently asserted assertions.
	foreach_handle_of_type(atom_space, "AssertionLink", 
	                       &QueryProcessor::do_assertion, this);

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
}

/**
 * Process an assertion fed into the system.
 * Currently, this ignores all assertions that are not queries.
 */
bool QueryProcessor::do_assertion(Handle h)
{
	printf ("duuuude found assertion handle=%p\n", h);

	// If this assertion is a query, try to answer it.
	RelexQuery rlx;
	if (rlx.is_query(h))
	{
		rlx.solve(atom_space, h);
	}
	atom_space->removeAtom(h);
	return false;
}

/* ======================= END OF FILE ==================== */
