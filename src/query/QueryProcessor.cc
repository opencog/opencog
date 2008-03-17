/**
 * QueryProcessor.cc
 *
 * Process user queries.
 *
 * XXX Currently, this is very crude scaffolding to interface
 * to the opencog server. It needs to be replaced/expanded as
 * appropriate.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "AtomSpace.h"
#include "CogServer.h"
#include "Foreach.h"
#include "MindAgent.h"
#include "QueryProcessor.h"
#include "FrameQuery.h"
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
	
	// Look for recently asserted assertions.
	foreach_handle_of_type(atom_space, "AssertionLink", 
	                       &QueryProcessor::do_assertion, this);

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	// usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
}

static int cnt = 0;

/**
 * Process an assertion fed into the system.
 * Currently, this ignores all assertions that are not queries.
 */
bool QueryProcessor::do_assertion(Handle h)
{
	cnt ++;
	printf ("duuuude found assertion %d handle=%p\n", cnt, h);

	// If this assertion is a query, try to answer it.
#if USE_RELEX_QUERY
	RelexQuery rlx;
	if (rlx.is_query(h))
	{
		rlx.solve(atom_space, h);
	}
#else
	FrameQuery frq;
	if (frq.is_query(h))
	{
		frq.solve(atom_space, h);
	}
#endif
	atom_space->removeAtom(h);
	return false;
}

/* ======================= END OF FILE ==================== */
