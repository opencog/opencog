/*
 * SchemeAdHoc.c
 *
 * Scheme adhoc callback into opencog
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/server/CogServer.h>

#include <opencog/nlp/question/SentenceQuery.h>
#include <opencog/nlp/question/TripleQuery.h>
#include <opencog/nlp/wsd/WordSenseProcessor.h>
#include <opencog/query/PatternMatch.h>
#include <opencog/reasoning/pln/PLNModule.h>

#include "SchemeSmob.h"

using namespace opencog;

/* ============================================================== */

/**
 * Dispatcher to invoke various miscellaneous C++ riff-raff from
 * scheme code. 
 */
SCM SchemeSmob::ss_ad_hoc(SCM command, SCM optargs)
{
	std::string cmdname = decode_string (command, "cog-ad-hoc", "string command name");

	if (0 == cmdname.compare("do-wsd"))
	{
		WordSenseProcessor wsp;
		wsp.use_threads(false);
		wsp.run_no_delay(&cogserver()); // XXX What an ugly interface. Alas.
		return SCM_BOOL_T;
	}

	// Run implication, assuming that the argument is a handle to 
	// an ImplicationLink
	if (0 == cmdname.compare("do-implication"))
	{
		// XXX we should also allow opt-args to be a list of handles
		Handle h = verify_handle(optargs, "cog-ad-hoc do-implication");

		AtomSpace *as = &atomspace();

		PatternMatch pm;
		pm.set_atomspace(as);
		Handle grounded_expressions = pm.crisp_logic_imply(h);
		return handle_to_scm(grounded_expressions);
	}

	// Store the single atom to the backing store hanging off the atom-space
	if (0 == cmdname.compare("store-atom"))
	{
		// XXX we should also allow opt-args to be a list of handles
		Handle h = verify_handle(optargs, "cog-ad-hoc store-atom");

		AtomSpace *as = &atomspace();
		as->storeAtom(h);
		return SCM_BOOL_T;
	}

	if (0 == cmdname.compare("fetch-atom"))
	{
		// XXX we should also allow opt-args to be a list of handles
		Handle h = verify_handle(optargs, "cog-ad-hoc fetch-atom");

		AtomSpace *as = &atomspace();
		h = as->fetchAtom(h);
		return handle_to_scm(h);
	}

	if (0 == cmdname.compare("fetch-incoming-set"))
	{
		// XXX we should also allow opt-args to be a list of handles
		Handle h = verify_handle(optargs, "cog-ad-hoc fetch-incoming-set");

		// The "true" flag here means "fetch resursive".
		AtomSpace *as = &atomspace();
		h = as->fetchIncomingSet(h, true);
		return handle_to_scm(h);
	}

	if (0 == cmdname.compare("question"))
	{
		Handle h = verify_handle(optargs, "cog-ad-hoc question");
		AtomSpace *as = &atomspace();

		SentenceQuery rlx;
		if (rlx.is_query(h))
		{
			rlx.solve(as, h);
			return SCM_BOOL_T;
		}
		return SCM_BOOL_F;
#if EXAMPLE_FRAME_QUERY_API
		FrameQuery frq;
		if (frq.is_query(h))
		{
			frq.solve(as, h);
		}
#endif

		return handle_to_scm(h);
	}
	if (0 == cmdname.compare("triple-question"))
	{
		Handle h = verify_handle(optargs, "cog-ad-hoc triple-question");
		AtomSpace *as = &atomspace();

		TripleQuery rlx;
		rlx.clear();
		rlx.add_triple(h);
		rlx.solve(as);

		return handle_to_scm(h);
	}
	return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Specify the target atom for PLN backward chaining inference.
 * Note: Use the cogserver commands to run the inference.
 */
SCM SchemeSmob::pln_bc (SCM starget, SCM ssteps)
{
	Handle h = verify_handle(starget, "pln-bc");
	int steps = scm_to_int(ssteps);

	Atom *a = TLB::getAtom(h);
	// We need to make a copy. Wish I could do this on stack ... 
	TruthValue *t = a->getTruthValue().clone();

//	setTarget(h);
	infer(h, steps);

	// Return true only if the truth value changed,
	// else return false.
	SCM rc = SCM_BOOL_T;
	if (*t == a->getTruthValue()) rc = SCM_BOOL_F;
	delete t;
	
	return rc;
}

#endif
/* ===================== END OF FILE ============================ */
