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

QueryModule::QueryModule(void)
{
#ifdef HAVE_GUILE
	define_scheme_primitive("cog-bind", &QueryModule::do_bindlink, this);
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
	AtomSpace *as = &atomspace();
	PatternMatch pm;
	pm.set_atomspace(as);
	Handle grounded_expressions = pm.bindlink(h);
	return grounded_expressions;
}
