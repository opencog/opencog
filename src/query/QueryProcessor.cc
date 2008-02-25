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

/**
 * Set pointer to Node, if the node name is "match_name".
 */
bool QueryProcessor::match_node_name(Atom *arel)
{
	Node *n = dynamic_cast<Node *>(arel);
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
		printf ("found query its %s\n", node->toString().c_str());
		varlist.push_back(TLB::getHandle(node));
	}
}

/**
 * Process an assertion fed into the system.
 * Currently, this ignores all assertions that are not queries.
 */
void QueryProcessor::do_assertion(Handle h)
{
	printf ("duuuude found assertion handle=%p\n", h);

	// Look for unbound query variables
	varlist.clear();
	foreach_outgoing_handle(h, &QueryProcessor::check_for_query, this);

	// If a query, try to answer it.
	if (0 != varlist.size())
	{
		PatternMatch pm;
		pm.match(h, varlist);
	}
}

/* ======================= END OF FILE ==================== */
