/*
 * PatternSCM.h
 *
 * Guile scheme bindings for the pattern matcher
 * Copyright (c) 2008, 2014 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_PATTERN_SCM_H
#define _OPENCOG_PATTERN_SCM_H

#include <opencog/atomspace/Handle.h>

namespace opencog {

class AtomSpace;

/// Wrapper class, to invoke pattern matcher from guile.
class PatternWrap
{
	private:
		Handle wrapper(Handle);
		Handle (*_func)(AtomSpace*, Handle);
		const char *_name;  // scheme name of the c++ function.
	public:
		PatternWrap(Handle (*)(AtomSpace*, Handle), const char*);
};

class PatternSCM
{
	private:
		static std::vector<PatternWrap*> _binders;
	public:
		PatternSCM(void);
		~PatternSCM();
};


}

#endif // _OPENCOG_PATTERN_SCM_H

