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
		AtomSpace *atom_space;

	public:
		WordSenseProcessor(void);
		virtual ~WordSenseProcessor();
		virtual void run(CogServer *server);
};
}

