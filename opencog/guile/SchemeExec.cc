/*
 * SchemeExec.c
 *
 * Execute ExecutionLink's
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/Link.h>

#include "SchemeEval.h"
#include "SchemeSmob.h"

using namespace opencog;

Handle SchemeEval::do_apply(const std::string &func, Handle varargs)
{
	per_thread_init();
printf("duuuude entery apply func = %s\n", func.c_str());
do_eval("(define (blah x y) (list \"you whooo\"))");
	SCM sfunc = scm_from_locale_symbol(func.c_str());

	Link *largs = dynamic_cast<Link *>(TLB::getAtom(varargs));
	if (NULL == largs) return Handle::UNDEFINED;

	const std::vector<Handle> &oset = largs->getOutgoingSet();

	SCM expr = SCM_EOL;
	size_t sz = oset.size();
	for (int i=sz-1; i>=0; i--)
	{
		Handle h = oset[i];
		SCM sh = SchemeSmob::handle_to_scm(h);
		expr = scm_cons(sh, expr);
	}
	expr = scm_cons(sfunc, expr);
	
printf("hooooooooooooooooooooooooooooooooooooooo\n");
std::string s = prt(expr);
printf("duuude will eval %s\n", s.c_str());

   SCM sr = do_scm_eval(expr);
printf("wwww hooooooooooooooooooooooooooooooooooooooo\n");
s = prt(sr);
printf("duuude result was %s\n", s.c_str());
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, sr))
	{
		fprintf(stderr, "Error: do_apply(): expecting handle for result!\n");
   	return Handle::UNDEFINED;
	}

	return SchemeSmob::scm_to_handle(sr);
}


#endif
/* ===================== END OF FILE ============================ */
