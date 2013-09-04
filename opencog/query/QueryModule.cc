/*
 * QueryModule.cc
 *
 * Load the query subsystem as a module.
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>

#include "PatternMatch.h"
#include "QueryModule.h"

using namespace opencog;

DECLARE_MODULE(QueryModule);

QueryModule::QueryModule(CogServer& cs) : Module(cs)
{
#ifdef HAVE_GUILE
	define_scheme_primitive("cog-bind", &QueryModule::do_bindlink, this);
	define_scheme_primitive("cog-bind-crisp", &QueryModule::do_crisp_bindlink, this);
#endif
}

QueryModule::~QueryModule()
{
}

void QueryModule::init(void)
{
}

/**
 * Run implication, assuming that the argument is a handle to
 * an BindLink containing variables and an ImplicationLink
 */
Handle QueryModule::do_bindlink(Handle h)
{
	// XXX we should also allow opt-args to be a list of handles
	AtomSpace *as = &_cogserver.getAtomSpace();
	PatternMatch pm;
	pm.set_atomspace(as);
	Handle grounded_expressions = pm.bindlink(h);
	return grounded_expressions;
}

/**
 * Run implication, assuming that the argument is a handle to
 * an BindLink containing variables and an ImplicationLink
 */
Handle QueryModule::do_crisp_bindlink(Handle h)
{
	// XXX we should also allow opt-args to be a list of handles
	AtomSpace *as = &_cogserver.getAtomSpace();
	PatternMatch pm;
	pm.set_atomspace(as);
	Handle grounded_expressions = pm.crisp_logic_bindlink(h);
	return grounded_expressions;
}
