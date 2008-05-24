/*
 * WordSenseProcessor.h
 *
 * Main entry point for word-sense disambiguation. This class inherits
 * from MindAgent; it scans for new input sentences, and then invokes
 * the disambiguation algorithms on it.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "MindAgent.h"
#include "Mihalcea.h"

namespace opencog {

class WordSenseProcessor : public MindAgent
{
	private:

		static void * thread_start(void *);

		int cnt;
		AtomSpace *atom_space;
		bool do_sentence(Handle h);

		Handle completion_handle;
		bool check_done(Handle h);

		Mihalcea *wsd;

	public:
		WordSenseProcessor(void);
		virtual ~WordSenseProcessor();
		virtual void run(CogServer *server);
};
}

