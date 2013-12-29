/*
 * SchemeExec.cc
 *
 * Execute ExecutionLink's
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include "SchemeEval.h"
#include "SchemeSmob.h"

using namespace opencog;

/**
 * do_apply -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the fuction func
 * is applied to them. If the function returns an atom handle, then
 * this is returned.
 */
Handle SchemeEval::do_apply(const std::string &func, Handle varargs)
{
	// Apply the function to the args
	SCM sresult = do_apply_scm (func, varargs);
	
	// If the result is a handle, return the handle.
	return SchemeSmob::scm_to_handle(sresult);
}

/**
 * do_apply_scm -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the fuction func
 * is applied to them. The SCM value returned by the function is returned.
 */
SCM SchemeEval::do_apply_scm( const std::string& func, Handle varargs )
{
	SCM sfunc = scm_from_locale_symbol(func.c_str());
	SCM expr = SCM_EOL;
	
	// If there were args, pass the args to the function.
	const std::vector<Handle> &oset = atomspace->getOutgoing(varargs);
		
	size_t sz = oset.size();
	for (int i=sz-1; i>=0; i--)
	{
		Handle h = oset[i];
		SCM sh = SchemeSmob::handle_to_scm(h);
		expr = scm_cons(sh, expr);
	}
	expr = scm_cons(sfunc, expr);
	return do_scm_eval(expr);
}

#endif
/* ===================== END OF FILE ============================ */
