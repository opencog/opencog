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

class PatternSCM
{
	private:
		Handle do_bindlink(Handle);
		Handle do_single_bindlink(Handle);
		Handle do_crisp_bindlink(Handle);
		static PatternSCM* _inst;
		void init(void);
	public:
		PatternSCM();
		~PatternSCM();
};

}

#endif // _OPENCOG_PATTERN_SCM_H

