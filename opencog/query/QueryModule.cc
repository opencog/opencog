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

QueryModule::QueryModule(CogServer& cs) : Module(cs), _pat(NULL)
{
}

QueryModule::~QueryModule()
{
	delete _pat;
}

void QueryModule::init(void)
{
	_pat = new PatternSCM();
}
