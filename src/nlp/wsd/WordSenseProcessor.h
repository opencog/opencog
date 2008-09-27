/*
 * WordSenseProcessor.h
 *
 * Main entry point for word-sense disambiguation. This class inherits
 * from MindAgent; it scans for new input sentences, and then invokes
 * the disambiguation algorithms on it.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <pthread.h>

#include "MindAgent.h"
#include "Mihalcea.h"

namespace opencog {

class WordSenseProcessor : public MindAgent
{
	private:

		pthread_t worker;
		static void * thread_start(void *);
		std::queue<Handle> work_queue;
		pthread_mutex_t queue_lock;
		bool thread_running;
		void work_thread(void);

		int cnt;
		AtomSpace *atom_space;
		bool do_document(Handle h);

		Handle start_handle;
		Handle completion_handle;
		bool check_start(Handle h);

		Mihalcea *wsd;

	public:
		WordSenseProcessor(void);
		virtual ~WordSenseProcessor();
		virtual void run(CogServer *server);
};
}

