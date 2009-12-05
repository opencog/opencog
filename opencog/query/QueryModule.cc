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
	define_scheme_primitive("do-implication", &QueryModule::do_implication, this);
	define_scheme_primitive("do-varscope", &QueryModule::do_varscope, this);
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
 * an ImplicationLink. XXX DEPRECATED: Use varscope below!
 */
Handle QueryModule::do_implication(Handle h)
{
	// XXX we should also allow opt-args to be a list of handles
	AtomSpace *as = &atomspace();
	PatternMatch pm;
	pm.set_atomspace(as);
	Handle grounded_expressions = pm.varscope(h);
	return grounded_expressions;
}

/**
 * Run implication, assuming that the argument is a handle to
 * an VarScopeLink containing variables and an ImplicationLink
 */
Handle QueryModule::do_varscope(Handle h)
{
	// XXX we should also allow opt-args to be a list of handles
	AtomSpace *as = &atomspace();
	PatternMatch pm;
	pm.set_atomspace(as);
	Handle grounded_expressions = pm.varscope(h);
	return grounded_expressions;
}
