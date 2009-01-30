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
		Handle grounded_expressions = pm.imply(h);
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
	return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Specify the target atom for PLN backward chaining inference.
 * Note: Use the cogserver commands to run the inference.
 */
SCM SchemeSmob::pln_bc (SCM satom)
{
	Handle h = verify_handle(satom, "pln-bc");

    setTarget(h);

    return SCM_BOOL_T;
}

#endif
/* ===================== END OF FILE ============================ */
