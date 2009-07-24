/*
 * SchemeExec.cc
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

/**
 * do_apply -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the fuction func
 * is applied to them. If the function returns an atom handle, then
 * this is returned, in turn.
 */
Handle SchemeEval::do_apply(const std::string &func, Handle varargs)
{
	per_thread_init();
	SCM sfunc = scm_from_locale_symbol(func.c_str());
	SCM expr = sfunc;

	// If there were args, pass the args to the function.
	Link *largs = dynamic_cast<Link *>(TLB::getAtom(varargs));
	if (largs)
	{
		const std::vector<Handle> &oset = largs->getOutgoingSet();

		expr = SCM_EOL;
		size_t sz = oset.size();
		for (int i=sz-1; i>=0; i--)
		{
			Handle h = oset[i];
			SCM sh = SchemeSmob::handle_to_scm(h);
			expr = scm_cons(sh, expr);
		}
		expr = scm_cons(sfunc, expr);
	}

	// Apply the function to the args
   SCM sresult = do_scm_eval(expr);

	// If the result is a handle, return the handle.
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, sresult))
	{
   	return Handle::UNDEFINED;
	}
	return SchemeSmob::scm_to_handle(sresult);
}

#endif
/* ===================== END OF FILE ============================ */
