/*
 * SchemeExec.h
 *
 * Evaluate scheme expressions specified with ExecutionLink's
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifndef OPENCOG_SCHEME_EXEC_H
#define OPENCOG_SCHEME_EXEC_H
#ifdef HAVE_GUILE

#include <string>
#include <opencog/atomspace/types.h>
#include "SchemeEval.h"

namespace opencog {

class SchemeExec
{
	private:
		SchemeEval evaluator;

	public:
		SchemeExec(void);
		~SchemeExec();
		Handle eval(const char *, Handle args);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_EXEC_H */
