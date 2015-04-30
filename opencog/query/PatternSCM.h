/*
 * PatternSCM.h
 *
 * Guile scheme bindings for the pattern matcher
 * Copyright (c) 2008, 2014, 2015 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_PATTERN_SCM_H
#define _OPENCOG_PATTERN_SCM_H

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/TruthValue.h>

namespace opencog {

class AtomSpace;

/// Wrapper class, to invoke pattern matcher from guile.
class PatternWrap
{
	private:
		Handle (*_func)(AtomSpace*, const Handle&);
		Handle wrapper(Handle);

		TruthValuePtr (*_pred)(AtomSpace*, const Handle&);
		TruthValuePtr prapper(Handle);

		const char *_name;  // scheme name of the c++ function.
	public:
		PatternWrap(Handle (*)(AtomSpace*, const Handle&), const char*);
		PatternWrap(TruthValuePtr (*)(AtomSpace*, const Handle&), const char*);
};

class PatternSCM
{
	private:
		static void* init_in_guile(void*);
		static void init_in_module(void*);
		static std::vector<PatternWrap*> _binders;
	public:
		PatternSCM(void);
		~PatternSCM();
};

}

extern "C" {
void opencog_query_init(void);
};

#endif // _OPENCOG_PATTERN_SCM_H

