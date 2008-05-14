/*
 * WordSenseProcoessor.h
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "MindAgent.h"

namespace opencog {

class WordSenseProcessor : public MindAgent
{
	private:
		int cnt;
		AtomSpace *atom_space;
		bool do_sentence(Handle h);

	public:
		WordSenseProcessor(void);
		virtual ~WordSenseProcessor();
		virtual void run(CogServer *server);
};
}

