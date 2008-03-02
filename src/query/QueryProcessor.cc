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
#include "Node.h"
#include "PatternMatch.h"
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
 * Set pointer to Node, if the node name is "match_name".
 */
bool QueryProcessor::match_node_name(Atom *atom)
{
	Node *n = dynamic_cast<Node *>(atom);
	if (n)
	{
		const std::string& name = n->getName();
		if (0 == strcmp(name.c_str(), match_name))
		{
			node = n;
			return true;
		}
	}
	return false;
}

/**
 * Search for queries
 */
bool QueryProcessor::check_for_query(Handle rel)
{
	match_name = "_$qVar";
	node = NULL;
	foreach_outgoing_atom(rel, &QueryProcessor::match_node_name, this);
	if (node)
	{
		// printf ("found query its %s\n", node->toString().c_str());
		varlist.push_back(TLB::getHandle(node));
	}
	return false;
}

/**
 * Process an assertion fed into the system.
 * Currently, this ignores all assertions that are not queries.
 */
bool QueryProcessor::do_assertion(Handle h)
{
	printf ("duuuude found assertion handle=%p\n", h);

	// Look for unbound query variables
	varlist.clear();
	foreach_outgoing_handle(h, &QueryProcessor::check_for_query, this);

	// If this assertion is a query, try to answer it.
	if (0 != varlist.size())
	{
		PatternMatch pm(atom_space);
		RelexQuery rlx;
		pm.filter(h);
		pm.match(&rlx);
	}
	atom_space->removeAtom(h);
	return false;
}

/* ======================= END OF FILE ==================== */
