/*
 * SchemeExec.cc
 *
 * Execute ExecutionOutputLink's
 * Copyright (c) 2009,2015 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#include <cstddef>
#include <libguile.h>
#include <opencog/execution/ExecutionOutputLink.h>
#include <opencog/execution/EvaluationLink.h>

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
Handle SchemeEval::do_apply(const std::string &func, Handle& varargs)
{
	// Apply the function to the args
	SCM sresult = do_apply_scm (func, varargs);

	// If the result is a handle, return the handle.
	return SchemeSmob::scm_to_handle(sresult);
}

static SCM thunk_scm_eval(void * expr)
{
	return scm_eval((SCM)expr, scm_interaction_environment());
}

/**
 * do_apply_scm -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the fuction func
 * is applied to them. The SCM value returned by the function is returned.
 */
SCM SchemeEval::do_apply_scm(const std::string& func, Handle& varargs )
{
	SCM sfunc = scm_from_utf8_symbol(func.c_str());
	SCM expr = SCM_EOL;

	// If there were args, pass the args to the function.
	const std::vector<Handle> &oset = atomspace->getOutgoing(varargs);

	// Iterate in reverse, because cons chains in reverse.
	size_t sz = oset.size();
	for (int i=sz-1; i>=0; i--)
	{
		SCM sh = SchemeSmob::handle_to_scm(oset[i]);
		expr = scm_cons(sh, expr);
	}
	expr = scm_cons(sfunc, expr);
	return do_scm_eval(expr, thunk_scm_eval);
}

/**
 * Executes an ExecutionOutputLink
 */
SCM SchemeSmob::ss_execute (SCM satom)
{
	Handle h = verify_handle(satom, "cog-execute!");

	if (h->getType() != EXECUTION_OUTPUT_LINK)
	{
		scm_wrong_type_arg_msg("cog-execute!", 1, satom,
			"ExecutionOutputLink opencog cog-execute!!");
	}

	AtomSpace* atomspace = ss_get_env_as("cog-execute!");
	// do_execute() may throw a C++ exception in various cases:
	// e.g. if it names a non-existant function, or a function
	// with syntax errors.
	try
	{
		return handle_to_scm(ExecutionOutputLink::do_execute(atomspace, h));
	}
	catch (const std::exception& ex)
	{
		SchemeSmob::throw_exception(ex.what(), "cog-execute!");
	}
	catch (...)
	{
		SchemeSmob::throw_exception(NULL, "cog-execute!");
	}
	scm_remember_upto_here_1(satom);
	return SCM_EOL;
}

/**
 * Executes an EevaluationLink with a GPN in it.
 */
SCM SchemeSmob::ss_evaluate (SCM satom)
{
	Handle h = verify_handle(satom, "cog-evaluate!");

	if (h->getType() != EVALUATION_LINK)
	{
		scm_wrong_type_arg_msg("cog-evaluate!", 1, satom,
			"EvaluationLink opencog cog-evaluate!");
	}

	AtomSpace* atomspace = ss_get_env_as("cog-evaluate!");
	// do_evaluate() may throw a C++ exception in various cases:
	// e.g. if it names a non-existant function, or a function
	// with syntax errors.
	try
	{
		TruthValuePtr tvp = EvaluationLink::do_evaluate(atomspace, h);
		return take_tv(tvp->rawclone());
	}
	catch (const std::exception& ex)
	{
		SchemeSmob::throw_exception(ex.what(), "cog-evaluate!");
	}
	catch (...)
	{
		SchemeSmob::throw_exception(NULL, "cog-evaluate!");
	}
	scm_remember_upto_here_1(satom);
	return SCM_EOL;
}

#endif
/* ===================== END OF FILE ============================ */
