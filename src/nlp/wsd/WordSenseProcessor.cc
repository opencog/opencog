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
#include <pthread.h>

#include "AtomSpace.h"
#include "CogServer.h"
#include "Foreach.h"
#include "ForeachChaseLink.h"
#include "Link.h"
#include "MihalceaLabel.h"
#include "MindAgent.h"
#include "Node.h"
#include "WordSenseProcessor.h"

using namespace opencog;

// ----------------------------------------
WordSenseProcessor::WordSenseProcessor(void)
{
	pthread_mutex_init(&queue_lock, NULL);
	thread_running = false;
	cnt = 0;
	wsd = new Mihalcea();
}

WordSenseProcessor::~WordSenseProcessor()
{
	atom_space = NULL;
	delete wsd;
	wsd = NULL;
}

// ----------------------------------------

void * WordSenseProcessor::thread_start(void *data)
{
	WordSenseProcessor *wsp = (WordSenseProcessor *) data;
	wsp->work_thread();
	return NULL;
}

void WordSenseProcessor::work_thread(void)
{
	while (1)
	{
		pthread_mutex_lock(&queue_lock);
		if (work_queue.empty())
		{
			thread_running = false;
			return;
		}
		Handle h = work_queue.front();
		work_queue.pop();
		pthread_mutex_unlock(&queue_lock);

		wsd->process_sentence(h);
	}
}

// ----------------------------------------

void WordSenseProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();

	wsd->set_atom_space(atom_space);

	// Look for recently entered text
	atom_space->foreach_handle_of_type("SentenceNode",
	               &WordSenseProcessor::do_sentence, this);

	// XXX avoid lots and lots of polling! Polling is a real
	// cpu time-waster, so do it only infrequently.
	// Keep it to 1/20th of a second for interactivity.
	usleep(50 * 1000); // 50 milliseconds.
}

/**
 * Process a sentence fed into the system. This routine is called on
 * every sentence encountered in the system.  It looks for sentences
 * that are not tagged as being "finished."
 *
 * XXX This routine does not guarentee that sentences are found in
 * order. Once found, they are handled in serial order, in the order
 * in which they wre found. But they could have beenm found in "random"
 * order. This needs a long term fix.
 */
bool WordSenseProcessor::do_sentence(Handle h)
{
	// Obtain the handle which indicates that the processing of a
 	// sentence is complete. 
	Node node(CONCEPT_NODE, "#WSD_completed");
	completion_handle = atom_space->addRealAtom(node);

	// Look to see the the sentence is associated with the 
	// completion indicator. 
	bool rc = foreach_binary_link(h, INHERITANCE_LINK, &WordSenseProcessor::check_done, this);

	if (rc) return false;

	// If we are here, then there's a fresh sentence to work on.
	cnt++;
	printf ("WordSenseProcessor found sentence %d handle=%lx\n", cnt, (unsigned long) h);

	// Mark this sentence as being completed.
	std::vector<Handle> out;
	out.push_back(h);
	out.push_back(completion_handle);

	atom_space->addLink(INHERITANCE_LINK, out);

	// Now queue the sentence for actual processing.
	pthread_mutex_lock(&queue_lock);
	work_queue.push(h);

	if (false == thread_running)
	{
		thread_running = true;
		pthread_create (&worker, NULL, &WordSenseProcessor::thread_start, this);
	}

	pthread_mutex_unlock(&queue_lock);
	return false;
}

bool WordSenseProcessor::check_done(Handle h)
{
	if (h == completion_handle) return true;
	return false;
}

/* ======================= END OF FILE ==================== */
