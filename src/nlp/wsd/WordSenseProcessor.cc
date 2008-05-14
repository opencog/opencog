/**
 * WordSenseProcessor.cc
 *
 * Top-leve word-sense disambiguation entry point.
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
#include "ForeachChaseLink.h"
#include "Link.h"
#include "MindAgent.h"
#include "Node.h"
#include "WordSenseProcessor.h"

using namespace opencog;

// ----------------------------------------
WordSenseProcessor::WordSenseProcessor(void)
{
	cnt = 0;
}

WordSenseProcessor::~WordSenseProcessor()
{
	atom_space = NULL;
}

void WordSenseProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();

	// Look for recently entered text
	atom_space->foreach_handle_of_type("SentenceNode",
	               &WordSenseProcessor::do_sentence, this);

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	usleep(1000000);  // 1 second
	// usleep(10000);  // 10 millisecs == 100HZ
// printf("hellow wrold\n");
}

/**
 * Process a sentence fed into the system. This routine is called on
 * every sentence encountered in the system.
 */
bool WordSenseProcessor::do_sentence(Handle h)
{
	cnt++;

	// Obtain the handle which indicates that the processing of a
 	// sentence is complete. 
	Node node(CONCEPT_NODE, "#WSD_completed");
	completion_handle = atom_space->addRealAtom(node);

	// Look to see the the sentence is associated with the 
   // completion indicator. 
	ForeachChaseLink<WordSenseProcessor> lc;
	bool rc = lc.follow_link(h, INHERITANCE_LINK, 1, 0, &WordSenseProcessor::check_done, this);
	
	if (rc) return false;

	// If we are here, then there's a freash sentence to work on.
	printf ("duuuude found sentence %d handle=%lx\n", cnt, (unsigned long) h);

	// Mark this sentence as being completed.
	std::vector<Handle> out;
	out.push_back(h);
	out.push_back(completion_handle);

	atom_space->addLink(INHERITANCE_LINK, out);

	return false;
}

bool WordSenseProcessor::check_done(Handle h)
{
	if (h == completion_handle) return true;
	return false;
}

/* ======================= END OF FILE ==================== */
