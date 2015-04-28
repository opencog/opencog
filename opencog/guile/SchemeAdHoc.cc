/*
 * SchemeAdHoc.c
 *
 * Scheme adhoc callback into opencog
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>
#include "SchemeSmob.h"

using namespace opencog;

/* ============================================================== */

/**
 * Dispatcher to invoke various miscellaneous C++ riff-raff from
 * scheme code.
 */
SCM SchemeSmob::ss_ad_hoc(SCM command, SCM optargs)
{
	std::string cmdname = verify_string (command, "cog-ad-hoc", 2, "string command name");

#if REVIVE_OLD_C_CODE_FOR_SENTENCE_PATTERN_MATCH
	// As of September 2009, this code is "obsolete", in that all sentence
	// pattern matching is now done via scheme, and not C++. This makes
	// sentence-handling code simpler, easier to structure, and less
	// squirmy. The code below can be deleted after its allowed to smolder
	// for a while, say, sometime in 2010. We keep it just in case :-)
	if (0 == cmdname.compare("question")) {
		Handle h = verify_handle(optargs, "cog-ad-hoc question", 2);
		AtomSpace *as = &atomspace();

		SentenceQuery rlx;
		if (rlx.is_query(h)) {
			rlx.solve(as, h);
			return SCM_BOOL_T;
		}
		return SCM_BOOL_F;
#if EXAMPLE_FRAME_QUERY_API
		FrameQuery frq;
		if (frq.is_query(h)) {
			frq.solve(as, h);
		}
#endif

		return handle_to_scm(h);
	}
#endif // REVIVE_OLD_C_CODE_FOR_SENTENCE_PATTERN_MATCH
	return SCM_BOOL_F;
}

#endif
/* ===================== END OF FILE ============================ */
