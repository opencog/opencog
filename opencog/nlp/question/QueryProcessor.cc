/**
 * QueryProcessor.cc
 *
 * Process user queries.
 *
 * XXX Currently, this is very crude scaffolding to interface
 * to the opencog server. It needs to be eliminated and replaced
 * by scheme code.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "QueryProcessor.h"

#include <stdio.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/Node.h>
#include <opencog/nlp/question/FrameQuery.h>
#include <opencog/nlp/question/RelexQuery.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

// ----------------------------------------
QueryProcessor::QueryProcessor(AtomSpace *as)
{
	atom_space = as;
	cnt = 0;

	// Obtain the handle which indicates that the processing of a
 	// sentence is complete. 
	Node node(CONCEPT_NODE, "# Query processing completed");
	completion_handle = atom_space->addRealAtom(node);
}

QueryProcessor::~QueryProcessor()
{
	atom_space = NULL;
}

/**
 * Process a sentence fed into the system. This routine is called on
 * every sentence encountered in the system.
 * Currently, this ignores all assertions that are not queries.
 */
bool QueryProcessor::process_sentence(Handle h)
{
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
