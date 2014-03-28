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
 * Create SCM object wrapping the atomspace.
 * Do NOT take over memory management of it!
 */
SCM SchemeSmob::make_as (AtomSpace *as)
{
	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, as);
	SCM_SET_SMOB_FLAGS(smob, COG_AS);
	return smob;
}

/* ============================================================== */
/**
 * Take over memory management of an atom space
 */
SCM SchemeSmob::take_as (AtomSpace *as)
{
	scm_gc_register_collectable_memory (as,
	                 sizeof(*as), "opencog atomspace");
	return make_as(as);
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

/* ============================================================== */
/* Cast SCM to atomspace */

AtomSpace* SchemeSmob::ss_to_atomspace(SCM sas)
{
   scm_t_bits misctype = SCM_SMOB_FLAGS(sas);
	if (COG_AS != misctype)
		return NULL;
   return (AtomSpace *) SCM_SMOB_DATA(sas);
}

/* ============================================================== */
/**
 * Set the atomspace into the top-level interaction environment
 */

SCM SchemeSmob::atomspace_symbol;

void SchemeSmob::ss_set_env_as(AtomSpace *as)
{
	// Place the atomspace in the ... well, the top-level environment
	// I think this is the interaction-environment, at this point.
	// Not sure...
	scm_c_define("*-atomspace-*", make_as(as));
	atomspace_symbol = scm_c_lookup("*-atomspace-*");
	atomspace_symbol = scm_permanent_object(atomspace_symbol);
}

AtomSpace* SchemeSmob::ss_get_env_as(const char* subr)
{
	SCM ref = scm_variable_ref(atomspace_symbol);
	AtomSpace* as = ss_to_atomspace(ref);
	if (NULL == as)
		scm_out_of_range(subr, atomspace_symbol);
	return as;
}


#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
