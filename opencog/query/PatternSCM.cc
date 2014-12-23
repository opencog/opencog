/*
 * PatternSCM.cc
 *
 * Guile Scheme bindings for the pattern matcher.
 * Copyright (c) 2008, 2014 Linas Vepstas <linas@linas.org>
 */

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>

#include "PatternSCM.h"

using namespace opencog;

PatternWrap::PatternWrap(Handle (f)(AtomSpace*, Handle), const char* n)
	: _func(f), _name(n)
{
	define_scheme_primitive(_name, &PatternWrap::wrapper, this);
}

Handle PatternWrap::wrapper(Handle h)
{
#ifdef HAVE_GUILE
	// XXX we should also allow opt-args to be a list of handles
	AtomSpace *as = SchemeSmob::ss_get_env_as(_name);
	Handle grounded_expressions = _func(as, h);
	return grounded_expressions;
#else
	return Handle::UNDEFINED;
#endif
}
