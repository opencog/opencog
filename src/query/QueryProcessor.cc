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
#include "Node.h"
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

void QueryProcessor::do_assertion(Handle h)
{
	printf ("duuuude found assertion handle=%p\n", h);
	Atom *atom = TLB::getAtom(h);
	const std::vector<Handle> &vh = atom->getOutgoingSet();

	bool found = false;
	for (size_t i=0; i<vh.size(); i++)
	{
		Handle rel = vh[i];
		found = is_qvar(rel);
		if (found)
		{
			printf ("duuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuude\n");
		}
		Atom *arel = TLB::getAtom(rel);
		printf ("its %s\n", arel->toString().c_str());
	}
}

/**
 * Return true if the handle is a link, and this link
 * contains a node whose name is "_$qVar"
 */
bool QueryProcessor::is_qvar(Handle h)
{
	Atom *atom = TLB::getAtom(h);
	const std::vector<Handle> &vh = atom->getOutgoingSet();

	for (size_t i=0; i<vh.size(); i++)
	{
		Handle rel = vh[i];
		Atom *arel = TLB::getAtom(rel);
		Node *n = dynamic_cast<Node *>(arel);
		if (n)
		{
			const std::string& name = n->getName();
			if (0 == strcmp(name.c_str(), "_$qVar")) return true;
		}
	}

	return false;
}
