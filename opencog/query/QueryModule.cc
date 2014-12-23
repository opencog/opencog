/*
 * QueryModule.cc
 *
 * Load the query subsystem as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "BindLink.h"
#include "PatternMatch.h"
#include "PatternSCM.h"
#include "QueryModule.h"

using namespace opencog;

DECLARE_MODULE(QueryModule);

QueryModule::QueryModule(CogServer& cs) : Module(cs)
{
}

QueryModule::~QueryModule()
{
	foreach(PatternWrap *pw: _binders)
		delete pw;
}

void QueryModule::init(void)
{
	// Run implication, assuming that the argument is a handle to
	// an BindLink containing variables and an ImplicationLink.
	_binders.push_back(new PatternWrap(bindlink, "cog-bind"));

	// Identical to do_bindlink above, except that it only returns the
	// first match.
	_binders.push_back(new PatternWrap(single_bindlink, "cog-bind-single"));

	// Run implication, assuming that the argument is a handle to
	// an BindLink containing variables and an ImplicationLink
	_binders.push_back(new PatternWrap(crisp_logic_bindlink, "cog-bind-crisp"));

	// Mystery function
	_binders.push_back(new PatternWrap(pln_bindlink, "cog-bind-pln"));
}
