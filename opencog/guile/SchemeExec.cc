/*
 * SchemeExec.c
 *
 * Execute ExecutionLink's
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include "SchemeEval.h"

#include <libguile.h>

#include "SchemeSmob.h"

using namespace opencog;

Handle SchemeEval::do_apply(const std::string &func, Handle varargs)
{
printf("duuuude entery apply func = %s\n", func.c_str());
do_eval("(define (blah) (list \"you whooo\"))");
	SCM sfunc = scm_from_locale_symbol(func.c_str());
	sfunc = scm_list_1(sfunc);

	
   do_scm_eval(sfunc);
   return Handle::UNDEFINED;
}


#endif
/* ===================== END OF FILE ============================ */
