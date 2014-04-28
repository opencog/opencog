/*
 * SchemeSmobMisc.c
 *
 * Scheme small objects (SMOBS) garbage-collection methods
 *
 * Copyright (c) 2008,2009,2014 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/TruthValue.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

SCM SchemeSmob::mark_misc(SCM misc_smob)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(misc_smob);

	switch (misctype)
	{
		case COG_HANDLE: // Nothing to do here ...
		case COG_TV: // Nothing to do here ...
		case COG_AS: // Nothing to do here ...
		case COG_AV: // Nothing to do here ...
		case COG_EXTEND: // Nothing to do here ...
			return SCM_BOOL_F;

		// I don't get it .. started seeing these recently. I'm just
		// going to silently ignore thse, for now, don't know what
		// they mean. XXX TODO figure out and fix if needed. Or document.
		case 0:
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
		case COG_AS:
		{
			AtomSpace *as = (AtomSpace *) SCM_SMOB_DATA(node);
			auto has = deleteable_as.find(as);
			if (deleteable_as.end() != has)
			{
				deleteable_as.erase(has);
				scm_gc_unregister_collectable_memory (as,
			                  sizeof(*as), "opencog atomspace");
				delete as;
			}
			return 0;
		}
		case COG_AV:
			AttentionValue *av;
			av = (AttentionValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (av,
			                  sizeof(*av), "opencog av");
			delete av;
			return 0;

		case COG_HANDLE:
			Handle* hp;
			hp = (Handle*) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (hp,
			                  sizeof(*hp), "opencog handle");
			delete hp;
			return 0;

		case COG_TV:
			TruthValue *tv;
			tv = (TruthValue *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (tv,
			                  sizeof(*tv), "opencog tv");
			delete tv;
			return 0;

		case COG_EXTEND:
			PrimitiveEnviron *pe;
			pe = (PrimitiveEnviron *) SCM_SMOB_DATA(node);
			scm_gc_unregister_collectable_memory (pe,
			                  pe->get_size(), "opencog primitive environ");
			delete pe;
			return 0;

		default:
			fprintf(stderr, "Error: opencog-guile: "
			        "don't know how to free this type: %d\n",
			        (int) misctype);
			break;
	}
	return 0;
}

/* ============================================================== */

std::string SchemeSmob::misc_to_string(SCM node)
{
	scm_t_bits misctype = SCM_SMOB_FLAGS(node);
	switch (misctype)
	{
		case COG_AS:
			return as_to_string((AtomSpace *) SCM_SMOB_DATA(node));

		case COG_AV:
			return av_to_string((AttentionValue *) SCM_SMOB_DATA(node));

		case COG_HANDLE:
			return handle_to_string(node);

		case COG_TV:
			return tv_to_string((TruthValue *) SCM_SMOB_DATA(node));

		case COG_EXTEND:
		{
			// return "#<opencog extension>\n";
			// Hmm. Is this really the right thing to return ?? I'm not sure ..
			PrimitiveEnviron * pe = (PrimitiveEnviron *) SCM_SMOB_DATA(node);
			return pe->get_name();
		}
		default:
			return "#<unknown opencog type>\n";
	}
	return "";
}

int SchemeSmob::print_misc(SCM node, SCM port, scm_print_state * ps)
{
	std::string str = misc_to_string(node);
	scm_puts (str.c_str(), port);
	return 1; //non-zero means success
}

/* ============================================================== */

#endif /* HAVE_GUILE */
/* ===================== END OF FILE ============================ */
