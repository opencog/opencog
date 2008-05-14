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
#include "MindAgent.h"
#include "WordSenseProcessor.h"

using namespace opencog;

// ----------------------------------------
WordSenseProcessor::WordSenseProcessor(void)
{
}

WordSenseProcessor::~WordSenseProcessor()
{
	atom_space = NULL;
}

void WordSenseProcessor::run(CogServer *server)
{
	atom_space = server->getAtomSpace();

	/* XXX HACK ALERT -- no scheduling, so just sleep */
	// usleep(1000000);  // 1 second
	usleep(10000);  // 10 millisecs == 100HZ
// printf("hellow wrold\n");
}

/* ======================= END OF FILE ==================== */
