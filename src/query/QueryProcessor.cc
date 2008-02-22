
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
	Type qtype = ClassServer::getType("ConceptNode");
	Handle h = as->getHandle(qtype, "test");

	printf ("found handle =%p\n", h);

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
}

