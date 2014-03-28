/*
 * SchemeSmobAS.c
 *
 * Scheme small objects (SMOBS) for atom spaces.
 *
 * Copyright (c) 2008,2009,2014 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

/* ============================================================== */

std::string SchemeSmob::as_to_string(const AtomSpace *as)
{
#define BUFLEN 120
	char buff[BUFLEN];

	snprintf(buff, BUFLEN, "#<atomspace %p>", as);
	return buff;
}

/* ============================================================== */
/**
 * Take over memory management of an atom space
 */
SCM SchemeSmob::take_as (AtomSpace *as)
{
	scm_gc_register_collectable_memory (as,
	                 sizeof(*as), "opencog atomspace");

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, as);
	SCM_SET_SMOB_FLAGS(smob, COG_AS);
	return smob;
}

/* ============================================================== */
/**
 * Create a new atom space.
 */
SCM SchemeSmob::ss_new_as (void)
{
	AtomSpace *as = new AtomSpace();
	return take_as(as);
}

/* ============================================================== */
/**
 * Return true if the scm is an atom space
 */
SCM SchemeSmob::ss_as_p (SCM s)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
	{
		scm_t_bits misctype = SCM_SMOB_FLAGS(s);
		switch (misctype)
		{
			case COG_AS:
				return SCM_BOOL_T;

			default:
				return SCM_BOOL_F;
		}
	}
	return SCM_BOOL_F;
}

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
