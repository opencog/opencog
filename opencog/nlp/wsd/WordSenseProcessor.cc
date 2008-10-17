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

#include "WordSenseProcessor.h"

#include <stdio.h>
#include <pthread.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/nlp/wsd/MihalceaLabel.h>
#include <opencog/server/Agent.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()
{
    return WordSenseProcessor::info().id.c_str();
}

extern "C" Module* opencog_module_load()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(WordSenseProcessor::info().id, &WordSenseProcessor::factory);
    return static_cast<WordSenseProcessor*>(cogserver.createAgent(WordSenseProcessor::info().id, true));
}

extern "C" void opencog_module_unload(Module* module)
{
    WordSenseProcessor* agent = static_cast<WordSenseProcessor*>(module);
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.stopAgent(agent);
    delete agent;
}

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
	Node node(DISTINGUISHED_NODE, "#WSD_completed");
	completion_handle = atom_space->addRealAtom(node);

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

void WordSenseProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();

	wsd->set_atom_space(atom_space);

	// Look for recently entered text
	atom_space->foreach_handle_of_type("DocumentNode",
	               &WordSenseProcessor::do_document, this);

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
	Node node(DISTINGUISHED_NODE, "#WSD_started");
	start_handle = atom_space->addRealAtom(node);

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

	if (false == thread_running)
	{
		thread_running = true;
		pthread_create (&worker, NULL, &WordSenseProcessor::thread_start, this);
	}

	pthread_mutex_unlock(&queue_lock);
	return false;
}

bool WordSenseProcessor::check_start(Handle h)
{
	if (h == start_handle) return true;
	return false;
}

/* ======================= END OF FILE ==================== */
