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

std::set<AtomSpace*> SchemeSmob::deleteable_as;

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
	deleteable_as.insert(as);
	scm_gc_register_collectable_memory (as,
	                 sizeof(*as), "opencog atomspace");
	return make_as(as);
}

/* ============================================================== */
/**
 * Create a new atom space.  The parent argument might not
 * be present -- its optional.
 */
SCM SchemeSmob::ss_new_as (SCM s)
{
	AtomSpace *parent = ss_to_atomspace(s);

	AtomSpace *as = new AtomSpace(parent);
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
	if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sas))
		return NULL;

   scm_t_bits misctype = SCM_SMOB_FLAGS(sas);
	if (COG_AS != misctype)
		return NULL;

   return (AtomSpace *) SCM_SMOB_DATA(sas);
}

/* ============================================================== */
/**
 * Return current atomspace for this dynamic state.
 */
SCM SchemeSmob::atomspace_fluid;

SCM SchemeSmob::ss_get_as (void)
{
	return scm_fluid_ref(atomspace_fluid);
}

/**
 * Set the current atomspace for this dynamic state.
 * Return the previous atomspace.
 */
SCM SchemeSmob::ss_set_as (SCM s)
{
	if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, s))
		return SCM_BOOL_F;

	if (COG_AS != SCM_SMOB_FLAGS(s))
		return SCM_BOOL_F;

	SCM old_as = scm_fluid_ref(atomspace_fluid);
	scm_fluid_set_x(atomspace_fluid, s);
	return old_as;
}

/* ============================================================== */
/**
 * Set the atomspace into the top-level interaction environment
 * Since its held in a fluid, it can have a different value in each
 * thread, so that different threads can use different atomspaces,
 * all at the same time.
 */

void SchemeSmob::ss_set_env_as(AtomSpace *as)
{
	scm_fluid_set_x(atomspace_fluid, make_as(as));
}

AtomSpace* SchemeSmob::ss_get_env_as(const char* subr)
{
	SCM ref = scm_fluid_ref(atomspace_fluid);
	AtomSpace* as = ss_to_atomspace(ref);
	if (NULL == as)
		scm_out_of_range(subr, atomspace_fluid);
	return as;
}


/* ============================================================== */

/**
 * Search for an atomspace in a list of values.
 * Return the atomspace if found, else return null.
 * Throw errors if the list is not stictly just key-value pairs
 */
AtomSpace* SchemeSmob::get_as_from_list(SCM slist)
{
	while (scm_is_pair(slist))
	{
		SCM sval = SCM_CAR(slist);
		AtomSpace* as = ss_to_atomspace(sval);
		if (as) return as;
		slist = SCM_CDR(slist);
	}

	return NULL;
}


#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
