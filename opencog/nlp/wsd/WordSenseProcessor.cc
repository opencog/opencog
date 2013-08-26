/**
 * WordSenseProcessor.cc
 *
 * Top-level word-sense disambiguation entry point.
 *
 * XXX Currently, this is very crude scaffolding to interface
 * to the opencog server. There's stuff in here that is no longer used.
 * It needs to be scrapped/replaced/expanded as appropriate.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <stdio.h>
#include <pthread.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/nlp/wsd/MihalceaLabel.h>
#include <opencog/server/CogServer.h>

#include "WordSenseProcessor.h"

using namespace opencog;

DECLARE_MODULE(WordSenseProcessor);

void WordSenseProcessor::init(void)
{
}

// ----------------------------------------

WordSenseProcessor::WordSenseProcessor(CogServer& cs) : Module(cs)
{
	pthread_mutex_init(&queue_lock, NULL);
	thread_running = false;
	do_use_threads = true;
	cnt = 0;
	wsd = new Mihalcea();
	atom_space = &_cogserver.getAtomSpace();
	wsd->set_atom_space(atom_space);

#ifdef HAVE_GUILE
	define_scheme_primitive("run-wsd", &WordSenseProcessor::run_wsd, this);
#endif
}

WordSenseProcessor::~WordSenseProcessor()
{
	atom_space = NULL;
	delete wsd;
	wsd = NULL;
}

void WordSenseProcessor::use_threads(bool use)
{
	do_use_threads = use;
}

void WordSenseProcessor::run_wsd(void)
{
	do_use_threads = false;
	run_no_delay();
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
	completion_handle = atom_space->addNode(ANCHOR_NODE, "#WSD_completed");

	while (1)
	{
		pthread_mutex_lock(&queue_lock);
		if (work_queue.empty())
		{
			thread_running = false;
			pthread_mutex_unlock(&queue_lock);
			return;
		}
		Handle h = work_queue.front();
		work_queue.pop();
		pthread_mutex_unlock(&queue_lock);

		wsd->process_document(h);

		// Mark this document as being completed.
		std::vector<Handle> out;
		out.push_back(h);
		out.push_back(completion_handle);
		atom_space->addLink(INHERITANCE_LINK, out);
	}
}

// ----------------------------------------

void WordSenseProcessor::run_no_delay()
{
	// Look for recently entered text
	atom_space->foreach_handle_of_type("DocumentNode",
	               &WordSenseProcessor::do_document, this);
}

void WordSenseProcessor::run()
{
	run_no_delay();

	// XXX we are being called too often. this needs to be fixed.
	// in truth, should only poll on new input.
	usleep(50*1000);
}

/**
 * Process a document fed into the system. A document is taken to be
 * an ordered list of sentences, discussing some topic or set of
 * connected ideas.  The sentences composing the document are handled
 * in order.
 */
bool WordSenseProcessor::do_document(Handle h)
{
	// Obtain the handle which indicates that the WSD processing of a
 	// document has started.
	start_handle = atom_space->addNode(ANCHOR_NODE, "#WSD_started");

	// Look to see if the document is associated with the
	// start indicator.
	bool rc = foreach_binary_link(h, INHERITANCE_LINK, &WordSenseProcessor::check_start, this);

	if (rc) return false;

	// If we are here, then there's a fresh document to work on.
	cnt++;
	printf ("WordSenseProcessor found document %d handle=%lx\n", cnt, h.value());

	// Mark this document as being started.
	std::vector<Handle> out;
	out.push_back(h);
	out.push_back(start_handle);
	atom_space->addLink(INHERITANCE_LINK, out);

	// Now queue the document for actual processing.
	pthread_mutex_lock(&queue_lock);
	work_queue.push(h);

	if (do_use_threads && (false == thread_running))
	{
		thread_running = true;
		pthread_create (&worker, NULL, &WordSenseProcessor::thread_start, this);
	}

	pthread_mutex_unlock(&queue_lock);

	// If not using threads, then process it now.
	if (false == do_use_threads)
	{
		work_thread();
	}
	return false;
}

bool WordSenseProcessor::check_start(Handle h)
{
	if (h == start_handle) return true;
	return false;
}

/* ======================= END OF FILE ==================== */
