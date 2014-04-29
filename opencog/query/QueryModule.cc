/*
 * QueryModule.cc
 *
 * Load the query subsystem as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "PatternSCM.h"
#include "QueryModule.h"

using namespace opencog;

DECLARE_MODULE(QueryModule);

QueryModule::QueryModule(CogServer& cs) : Module(cs)
{
	pat = NULL;
}

QueryModule::~QueryModule()
{
	delete pat;
}

void QueryModule::init(void)
{
	// Force the constructor to run, so that the scheme initialization
	// happens.
	pat = new PatternSCM();
}
