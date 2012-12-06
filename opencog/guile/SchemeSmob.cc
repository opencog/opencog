/*
 * SchemeSmob.c
 *
 * Scheme small objects (SMOBS) for opencog -- core functions.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include "SchemeSmob.h"

#include <libguile.h>

#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

/**
 * Two scheme smob types are used to implement the interface.
 *
 * The cog_handle_tag is used to store atom handles only.
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
 */

scm_t_bits SchemeSmob::cog_handle_tag;
scm_t_bits SchemeSmob::cog_misc_tag;
bool SchemeSmob::is_inited = false;
AtomSpace* SchemeSmob::atomspace = NULL;

void SchemeSmob::init(AtomSpace *as)
{
	// XXX It would be ever so slightly more correct to use
	// pthread_once() here, but that currently seems like overkill.
	if (!is_inited)
	{
		is_inited = true;
		init_smob_type();
		register_procs();
		atomspace = as;
	}
}

SchemeSmob::SchemeSmob(AtomSpace *as)
{
	// printf("atomspace at schemesmob creation is %p", as);
	init(as);
}

/* ============================================================== */

int SchemeSmob::print_atom(SCM node, SCM port, scm_print_state * ps)
{
	std::string str = handle_to_string(node);
	scm_puts (str.c_str(), port);
	return 1; //non-zero means success
}

SCM SchemeSmob::equalp_atom(SCM a, SCM b)
{
	// Two atoms are equal if their handles are the same.
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
	// a SMOB type for atom handles
	cog_handle_tag = scm_make_smob_type ("opencog-handle", sizeof (scm_t_bits));
	scm_set_smob_print (cog_handle_tag, print_atom);
	scm_set_smob_equalp (cog_handle_tag, equalp_atom);
	scm_set_smob_free (cog_handle_tag, free_atom);

	// A SMOB type for everything else
	cog_misc_tag = scm_make_smob_type ("opencog-misc", sizeof (scm_t_bits));
	scm_set_smob_print (cog_misc_tag, print_misc);
	scm_set_smob_mark (cog_misc_tag, mark_misc);
	scm_set_smob_free (cog_misc_tag, free_misc);
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
	scm_c_define_gsubr("cog-atom?",             1, 0, 1, C(ss_atom_p));
	scm_c_define_gsubr("cog-node?",             1, 0, 1, C(ss_node_p));
	scm_c_define_gsubr("cog-link?",             1, 0, 1, C(ss_link_p));

	// property setters
	scm_c_define_gsubr("cog-set-av!",           2, 0, 0, C(ss_set_av));
	scm_c_define_gsubr("cog-set-tv!",           2, 0, 0, C(ss_set_tv));
	scm_c_define_gsubr("cog-inc-vlti!",         1, 0, 0, C(ss_inc_vlti));
	scm_c_define_gsubr("cog-dec-vlti!",         1, 0, 0, C(ss_dec_vlti));

	// property getters
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
	scm_c_define_gsubr("cog-new-mtv",           2, 0, 0, C(ss_new_mtv));
	scm_c_define_gsubr("cog-set-vtv!",          3, 0, 0, C(ss_set_vtv));
	scm_c_define_gsubr("cog-tv?",               1, 0, 0, C(ss_tv_p));
	scm_c_define_gsubr("cog-stv?",              1, 0, 0, C(ss_stv_p));
	scm_c_define_gsubr("cog-ctv?",              1, 0, 0, C(ss_ctv_p));
	scm_c_define_gsubr("cog-itv?",              1, 0, 0, C(ss_itv_p));
	scm_c_define_gsubr("cog-mtv?",              1, 0, 0, C(ss_mtv_p));
	scm_c_define_gsubr("cog-tv->alist",         1, 0, 0, C(ss_tv_get_value));

	// Version handles
	scm_c_define_gsubr("cog-new-vh",            2, 0, 0, C(ss_new_vh));
	scm_c_define_gsubr("cog-vh?",               1, 0, 0, C(ss_vh_p));
	scm_c_define_gsubr("cog-vh->alist",         1, 0, 0, C(ss_vh_get_value));

	// Attention values
	scm_c_define_gsubr("cog-new-av",            3, 0, 0, C(ss_new_av));
	scm_c_define_gsubr("cog-av?",               1, 0, 0, C(ss_av_p));
	scm_c_define_gsubr("cog-av->alist",         1, 0, 0, C(ss_av_get_value));

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
