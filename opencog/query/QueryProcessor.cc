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

#include "QueryProcessor.h"

#include <stdio.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/Node.h>
#include <opencog/query/FrameQuery.h>
#include <opencog/query/RelexQuery.h>
#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

Factory<QueryProcessor, Agent> QueryProcessor::factory;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()
{
    return QueryProcessor::info().id.c_str();
}

extern "C" Module* opencog_module_load()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(QueryProcessor::info().id, &QueryProcessor::factory);
    return static_cast<QueryProcessor*>(cogserver.createAgent(QueryProcessor::info().id, true));
}

extern "C" void opencog_module_unload(Module* module)
{
    QueryProcessor* agent = static_cast<QueryProcessor*>(module);
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.stopAgent(agent);
    delete agent;
}

// ----------------------------------------
QueryProcessor::QueryProcessor(void)
{
	cnt = 0;
}

QueryProcessor::~QueryProcessor()
{
	atom_space = NULL;
}

void QueryProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();
	
	// Look for recently asserted assertions.
	atom_space->foreach_handle_of_type("SentenceNode", 
	                       &QueryProcessor::do_assertion, this);

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	// usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
}

/**
 * Process a sentence fed into the system. This routine is called on
 * every sentence encountered in the system.
 * Currently, this ignores all assertions that are not queries.
 */
bool QueryProcessor::do_assertion(Handle h)
{
	// Obtain the handle which indicates that the processing of a
 	// sentence is complete. 
	Node node(CONCEPT_NODE, "#Query_processing_completed");
	completion_handle = atom_space->addRealAtom(node);

	// Look to see the the sentence is associated with the 
	// completion indicator. 
	bool rc = foreach_binary_link(h, INHERITANCE_LINK, &QueryProcessor::check_done, this);
	
	if (rc) return false;

	// If we are here, then there's a fresh sentence to work on.
	cnt ++;
	printf ("Query Processor found sentence %d handle=%lx\n", cnt, h.value());

	// If this assertion is a query, try to answer it.
#define USE_RELEX_QUERY 1
#ifdef USE_RELEX_QUERY
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

	// Mark this sentence as being completed.
	std::vector<Handle> out;
	out.push_back(h);
	out.push_back(completion_handle);

	atom_space->addLink(INHERITANCE_LINK, out);

	return false;
}

bool QueryProcessor::check_done(Handle h)
{
	if (h == completion_handle) return true;
	return false;
}

/* ======================= END OF FILE ==================== */
