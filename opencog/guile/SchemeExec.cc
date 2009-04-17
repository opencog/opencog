/*
 * SchemeExec.c
 *
 * Evaluate scheme expressions specified with ExecutionLink's
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include "SchemeExec.h"

#include <opencog/util/platform.h>
#include "SchemeSmob.h"

using namespace opencog;


SchemeExec::SchemeExec(void)
{
}

SchemeExec::~SchemeExec()
{
}

/* ============================================================== */

/**
 * Execuate the expression
 */
Handle SchemeExec::eval(const char *func, Handle varargs)
{
printf("duuuude func = %s\n", func);
evaluator.eval("(define (blah) (list \"you whooo\"))");
SCM sfunc = scm_from_locale_symbol("blah");
sfunc = scm_list_1(sfunc);
	evaluator.eval(sfunc);
	return Handle::UNDEFINED;
}

#endif
/* ===================== END OF FILE ============================ */
