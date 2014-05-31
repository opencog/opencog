/*
 * SchemeSmob.c
 *
 * Scheme small objects (SMOBS) for opencog -- core functions.
 *
 * Copyright (c) 2008, 2013, 2104 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <libguile.h>

#include <opencog/atomspace/AtomSpace.h>
#include "SchemePrimitive.h"
#include "SchemeSmob.h"

using namespace opencog;

/**
 * Two scheme smob types are used to implement the interface.
 *
 * The cog_uuid_tag is used to store atom handles as uuids.
 * The cog_misc_tag is used to store all other structures, such
 * as truth values. It is assumed that these structures are all
 * ephemeral (garbage-collected); this is in contrast to handles,
 * which are never garbage collected. Thus, opencog atoms have a
 * concrete existence outside of the scheme shell. By contrast,
 * truth values created by the scheme shell are garbage collected
 * by the shell.
 *
 * The type of the "misc" structure is stored in the flag bits;
 * thus, handling is dispatched based on these flags.
 *
 * XXX There are currently *two* ways of storing atoms: as UUID's
 * and as Handle-based AtomPtr's. The first is compact, and is great
 * if the atom has not yet been instantiated in RAM.  Unfortunately,
 * any sort of references from it requires resolving the UUID into an
 * AtomPtr, which requires finding it in an RB-tree in the AtomTable,
 * which requires tacking a reader-lock on the table, which can be a
 * serious bottle-neck when running atom-manipulating algorithms in
 * scheme. Thus, the second type of storate: a fully resolved AtomPtr,
 * which can quickly access atom attributes. The UUID form is 'rare'
 * and is not currently needed.
 *
 * XXX TODO:
 * The cog_misc_tag should be replaced by a tag-per-class (i.e. we
 * should have a separate tag for handles, tv's, etc.) This would
 * simplify that code, and probably improve performance just a bit.
 */

scm_t_bits SchemeSmob::cog_uuid_tag;
scm_t_bits SchemeSmob::cog_misc_tag;
bool SchemeSmob::is_inited = false;

void SchemeSmob::init()
{
	// XXX It would be ever so slightly more correct to use
	// pthread_once() here, but that currently seems like overkill.
	if (!is_inited)
	{
		is_inited = true;
		init_smob_type();
		register_procs();

		atomspace_variable = scm_c_define("*-atomspace-*", make_as(NULL));
		atomspace_variable = scm_permanent_object(atomspace_variable);
	}
}

SchemeSmob::SchemeSmob()
{
	init();
}

/* ============================================================== */

int SchemeSmob::print_atom(SCM node, SCM port, scm_print_state * ps)
{
	std::string str = uuid_to_string(node);
	scm_puts (str.c_str(), port);
	return 1; //non-zero means success
}

SCM SchemeSmob::equalp_atom(SCM a, SCM b)
{
	// Two atoms are equal if their UUIDs are the same.
	if (SCM_SMOB_OBJECT(a) == SCM_SMOB_OBJECT(b)) return SCM_BOOL_T;
	return SCM_BOOL_F;
}

size_t SchemeSmob::free_atom(SCM node)
{
	// Nothing to do here; the atom handles are stored as
	// immediate values in the SMOB's.
	return 0;
}

void SchemeSmob::init_smob_type(void)
{
	// a SMOB type for atom uuids
	cog_uuid_tag = scm_make_smob_type ("opencog-uuid", sizeof (scm_t_bits));
	scm_set_smob_print (cog_uuid_tag, print_atom);
	scm_set_smob_equalp (cog_uuid_tag, equalp_atom);
	// scm_set_smob_free (cog_uuid_tag, free_atom);

	// A SMOB type for everything else
	cog_misc_tag = scm_make_smob_type ("opencog-misc", sizeof (scm_t_bits));
	scm_set_smob_print (cog_misc_tag, print_misc);
	scm_set_smob_equalp (cog_misc_tag, equalp_misc);
	// scm_set_smob_mark (cog_misc_tag, mark_misc);
	scm_set_smob_free (cog_misc_tag, free_misc);
}

/* ============================================================== */

SCM SchemeSmob::equalp_misc(SCM a, SCM b)
{
	// If they're not something we know about, let scheme sort it out.
	// (Actualy, this should never happen ...)
	if (not SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, a))
		return scm_equal_p(a, b);

	// If the types don't match, they can't be equal.
	scm_t_bits ta = SCM_SMOB_FLAGS(a);
	scm_t_bits tb = SCM_SMOB_FLAGS(b);
	if (ta != tb)
		return SCM_BOOL_F;

	switch (ta)
	{
		default: // Should never happen.
		case 0:  // Should never happen.
			return SCM_BOOL_F;
		case COG_AS:
		{
			AtomSpace* as = (AtomSpace *) SCM_SMOB_DATA(a);
			AtomSpace* bs = (AtomSpace *) SCM_SMOB_DATA(b);
			/* Just a simple pointer comparison */
			if (as == bs) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_AV:
		{
			AttentionValue* av = (AttentionValue *) SCM_SMOB_DATA(a);
			AttentionValue* bv = (AttentionValue *) SCM_SMOB_DATA(b);
			if (av == bv) return SCM_BOOL_T;
			if (*av == *bv) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_EXTEND:
		{
			// We compare pointers here, only.
			PrimitiveEnviron* av = (PrimitiveEnviron *) SCM_SMOB_DATA(a);
			PrimitiveEnviron* bv = (PrimitiveEnviron *) SCM_SMOB_DATA(b);
			if (av == bv) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_HANDLE:
		{
			Handle ha(scm_to_handle(a));
			Handle hb(scm_to_handle(b));
			if (ha == hb) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_TV:
		{
			TruthValue* av = (TruthValue *) SCM_SMOB_DATA(a);
			TruthValue* bv = (TruthValue *) SCM_SMOB_DATA(b);
			if (av == bv) return SCM_BOOL_T;
			if (*av == *bv) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
	}
}

/* ============================================================== */

#ifdef HAVE_GUILE2
 #define C(X) ((scm_t_subr) X)
#else
 #define C(X) ((SCM (*) ()) X)
#endif

void SchemeSmob::register_procs(void)
{
	scm_c_define_gsubr("cog-atom",              1, 0, 0, C(ss_atom));
	scm_c_define_gsubr("cog-handle",            1, 0, 0, C(ss_handle));
	scm_c_define_gsubr("cog-undefined-handle",  0, 0, 0, C(ss_undefined_handle));
	scm_c_define_gsubr("cog-new-node",          2, 0, 1, C(ss_new_node));
	scm_c_define_gsubr("cog-new-link",          1, 0, 1, C(ss_new_link));
	scm_c_define_gsubr("cog-node",              2, 0, 1, C(ss_node));
	scm_c_define_gsubr("cog-link",              1, 0, 1, C(ss_link));
	scm_c_define_gsubr("cog-delete",            1, 0, 0, C(ss_delete));
	scm_c_define_gsubr("cog-delete-recursive",  1, 0, 0, C(ss_delete_recursive));
	scm_c_define_gsubr("cog-purge",             1, 0, 0, C(ss_purge));
	scm_c_define_gsubr("cog-purge-recursive",   1, 0, 0, C(ss_purge_recursive));
	scm_c_define_gsubr("cog-atom?",             1, 0, 1, C(ss_atom_p));
	scm_c_define_gsubr("cog-node?",             1, 0, 1, C(ss_node_p));
	scm_c_define_gsubr("cog-link?",             1, 0, 1, C(ss_link_p));

	// property setters on atoms
	scm_c_define_gsubr("cog-set-av!",           2, 0, 0, C(ss_set_av));
	scm_c_define_gsubr("cog-set-tv!",           2, 0, 0, C(ss_set_tv));
	scm_c_define_gsubr("cog-inc-vlti!",         1, 0, 0, C(ss_inc_vlti));
	scm_c_define_gsubr("cog-dec-vlti!",         1, 0, 0, C(ss_dec_vlti));

	// property getters on atoms
	scm_c_define_gsubr("cog-name",              1, 0, 0, C(ss_name));
	scm_c_define_gsubr("cog-type",              1, 0, 0, C(ss_type));
	scm_c_define_gsubr("cog-arity",             1, 0, 0, C(ss_arity));
	scm_c_define_gsubr("cog-incoming-set",      1, 0, 0, C(ss_incoming_set));
	scm_c_define_gsubr("cog-outgoing-set",      1, 0, 0, C(ss_outgoing_set));
	scm_c_define_gsubr("cog-tv",                1, 0, 0, C(ss_tv));
	scm_c_define_gsubr("cog-av",                1, 0, 0, C(ss_av));

	// Truth-values
	scm_c_define_gsubr("cog-new-stv",           2, 0, 0, C(ss_new_stv));
	scm_c_define_gsubr("cog-new-ctv",           3, 0, 0, C(ss_new_ctv));
	scm_c_define_gsubr("cog-new-itv",           3, 0, 0, C(ss_new_itv));
	scm_c_define_gsubr("cog-tv?",               1, 0, 0, C(ss_tv_p));
	scm_c_define_gsubr("cog-stv?",              1, 0, 0, C(ss_stv_p));
	scm_c_define_gsubr("cog-ctv?",              1, 0, 0, C(ss_ctv_p));
	scm_c_define_gsubr("cog-itv?",              1, 0, 0, C(ss_itv_p));
	scm_c_define_gsubr("cog-tv->alist",         1, 0, 0, C(ss_tv_get_value));

	// Atom Spaces
	scm_c_define_gsubr("cog-new-atomspace",     0, 0, 0, C(ss_new_as));
	scm_c_define_gsubr("cog-atomspace?",        1, 0, 1, C(ss_as_p));

	// Attention values
	scm_c_define_gsubr("cog-new-av",            3, 0, 0, C(ss_new_av));
	scm_c_define_gsubr("cog-av?",               1, 0, 0, C(ss_av_p));
	scm_c_define_gsubr("cog-av->alist",         1, 0, 0, C(ss_av_get_value));

	// AttentionalFocus
	scm_c_define_gsubr("cog-af-boundary",       0, 0, 0, C(ss_af_boundary));
	scm_c_define_gsubr("cog-set-af-boundary!",  1, 0, 0, C(ss_set_af_boundary));
	scm_c_define_gsubr("cog-af",                0, 0, 0, C(ss_af));
    
	// ExecutionLinks
	scm_c_define_gsubr("cog-execute!",          1, 0, 0, C(ss_execute));

	// Atom types
	scm_c_define_gsubr("cog-get-types",         0, 0, 0, C(ss_get_types));
	scm_c_define_gsubr("cog-type?",             1, 0, 0, C(ss_type_p));
	scm_c_define_gsubr("cog-get-subtypes",      1, 0, 0, C(ss_get_subtypes));
	scm_c_define_gsubr("cog-subtype?",          2, 0, 0, C(ss_subtype_p));

	// Iterators
	scm_c_define_gsubr("cog-map-type",          2, 0, 0, C(ss_map_type));
}

#endif
/* ===================== END OF FILE ============================ */
