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

#include "SchemeSmob.h"

using namespace opencog;

/* ============================================================== */

/**
 * Dispatcher to invoke various miscellaneous C++ riff-raff from
 * scheme code. 
 */
SCM SchemeSmob::ss_ad_hoc(SCM command)
{
	std::string cmdname = decode_string (command, "cog-ad-hoc");

	if (0 == cmdname.compare("do-wsd"))
	{
		WordSenseProcessor wsp;
		wsp.use_threads(false);
		wsp.run_no_delay(CogServer::createInstance()); // XXX What an ugly interface. Alas.
		return SCM_BOOL_T;
	}

	return SCM_BOOL_F;
}

#endif
/* ===================== END OF FILE ============================ */
