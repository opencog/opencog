/*
 * SchemeEvaluate.cc
 *
 * Execute EvaluationLink's
 * Copyright (c) 2009,2015	Linas Vepstas <linasvepstas@gmail.com>
 *
 * Modeled after SchemeExec for ExecutionOutputLinks by:
 *							Curtis Faith <curtis.m.faith@gmail.com>
 */

#ifdef HAVE_GUILE

#include <cstddef>
#include <libguile.h>
#include <opencog/execution/EvaluationLink.h>

#include "SchemeEval.h"
#include "SchemeSmob.h"

using namespace opencog;

/**
 * Evaluates an EvaluationLink
 */
SCM SchemeSmob::ss_evaluate (SCM satom)
{
	AtomSpace* atomspace = ss_get_env_as("cog-evaluate");

	Handle h = verify_handle(satom, "cog-evaluate");

	if (h->getType() != EVALUATION_LINK)
	{
		scm_wrong_type_arg_msg("cog-evaluate", 1, satom,
			"EVALUATION_LINK opencog cog-evaluate");
	}

	// do_evaluate() may throw a C++ exception in various cases:
	// e.g. if the code to execute is python, and its names a
	// non-existant function ... or even if its scheme code with
	// a bug in it.
	try
	{
		TruthValuePtr tvp = EvaluationLink::do_evaluate(atomspace, h);

		TruthValuePtr new_tvp = tvp->clone();
		SCM stvp = take_tv(new_tvp.get());
		return stvp;
	}
	catch (const std::exception& ex)
	{
		SchemeSmob::throw_exception(ex.what(), "cog-evaluate");
	}
	catch (...)
	{
		SchemeSmob::throw_exception(NULL, "cog-evaluate");
	}
	scm_remember_upto_here_1(satom);
	return SCM_EOL;
}

#endif
/* ===================== END OF FILE ============================ */
