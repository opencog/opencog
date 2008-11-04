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

SCM SchemeSmob::ss_ad_hoc(SCM command)
{
	std::string name = decode_string (command, "cog-ad-hoc");

	return SCM_BOOL_F;
}

#endif
/* ===================== END OF FILE ============================ */
