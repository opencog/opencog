/*
 * SchemeSmobMisc.c
 *
 * Scheme small objects (SMOBS) garbage-collection methods
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/VersionHandle.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

SCM SchemeSmob::mark_misc(SCM misc_smob)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(misc_smob);

	switch (misctype)
	{
		case COG_TV: // Nothing to do here ...
		case COG_VH: // Nothing to do here ...
			return SCM_BOOL_F;
		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to mark this type: %d\n",
			        (int) misctype);
			break;
	}

	return SCM_BOOL_F;
}

/**
 * Free the memory associated with an opencog guile object.
 * This routine is called by the guile garbage collector, from time to
 * time. For testing purposes, you can force the garbage collector to
 * run by saying (gc), while, for stats, try (gc-stats) and
 * (gc-live-object-stats). The later should show both "opencog-handle"
 * and "opencog-misc" stats.
 */
size_t SchemeSmob::free_misc(SCM node)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);

	switch (misctype)
	{
		case COG_TV:
			TruthValue *tv;
			tv = (TruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (tv,
			                  sizeof(*tv), "opencog tv");
			delete tv;
			return 0;

		case COG_VH:
			VersionHandle *vh;
			vh = (VersionHandle *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (vh,
			                  sizeof(*vh), "opencog vh");
			delete vh;
			return 0;

		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to free this type: %d\n",
			        (int) misctype);
			break;
	}
	return 0;
}

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
