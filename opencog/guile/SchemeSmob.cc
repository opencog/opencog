/*
 * SchemeSmob.c
 *
 * Scheme small objects (SMOBS) for opencog -- core functions.
 *
 * Copyright (c) 2008, 2013, 2014, 2015 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include <cstddef>
#include <libguile.h>

#include <opencog/atomspace/AtomSpace.h>
#include "SchemePrimitive.h"
#include "SchemeSmob.h"

using namespace opencog;

/**
 * Just one scheme smob type is used to implement the interface.
 *
 * The cog_misc_tag is used to store all structures, such as atoms
 * and truth values. It is assumed that these structures are all
 * ephemeral (garbage-collected), including the Handles.  Note that
 * atoms in the atomspace have a concrete existence outside of the
 * scheme shell. By contrast, truth values created by the scheme
 * shell are garbage collected by the shell.
 *
 * The type of the "misc" structure is stored in the flag bits;
 * thus, handling is dispatched based on these flags.
 *
 * XXX TODO:
 * The cog_misc_tag should be replaced by a tag-per-class (i.e. we
 * should have a separate tag for handles, tv's, etc.) This would
 * simplify that code, and probably improve performance just a bit.
 */

scm_t_bits SchemeSmob::cog_misc_tag;
std::atomic_flag SchemeSmob::is_inited = ATOMIC_FLAG_INIT;
SCM SchemeSmob::_radix_ten;

void SchemeSmob::init()
{
	if (is_inited.test_and_set()) return;

	init_smob_type();
	register_procs();

	atomspace_fluid = scm_make_fluid();
	atomspace_fluid = scm_permanent_object(atomspace_fluid);
	_radix_ten = scm_from_int8(10);
}

SchemeSmob::SchemeSmob()
{
	init();
}

void opencog_guile_init(void)
{
	SchemeSmob::init();
}

/* ============================================================== */

void SchemeSmob::init_smob_type(void)
{
	// A SMOB type for everything, incuding atoms.
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
			scm_remember_upto_here_1(a);
			scm_remember_upto_here_1(b);
			/* Just a simple pointer comparison */
			if (as == bs) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_AV:
		{
			AttentionValue* av = (AttentionValue *) SCM_SMOB_DATA(a);
			AttentionValue* bv = (AttentionValue *) SCM_SMOB_DATA(b);
			scm_remember_upto_here_1(a);
			scm_remember_upto_here_1(b);
			if (av == bv) return SCM_BOOL_T;
			if (*av == *bv) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
		case COG_EXTEND:
		{
			// We compare pointers here, only.
			PrimitiveEnviron* av = (PrimitiveEnviron *) SCM_SMOB_DATA(a);
			PrimitiveEnviron* bv = (PrimitiveEnviron *) SCM_SMOB_DATA(b);
			scm_remember_upto_here_1(a);
			scm_remember_upto_here_1(b);
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
			scm_remember_upto_here_1(a);
			scm_remember_upto_here_1(b);
			if (av == bv) return SCM_BOOL_T;
			if (*av == *bv) return SCM_BOOL_T;
			return SCM_BOOL_F;
		}
	}
}

/* ============================================================== */

void SchemeSmob::throw_exception(const char *msg, const char * func)
{
	if (msg) {
		// Should we even bother to log this?
		logger().info("Guile caught C++ exception: %s", msg);

		// scm_misc_error(fe->get_name(), msg, SCM_EOL);
		scm_throw(
			scm_from_utf8_symbol("C++-EXCEPTION"),
			scm_cons(
				scm_from_utf8_string(func),
				scm_cons(
					scm_from_utf8_string(msg),
					SCM_EOL)));
		// Hmm. scm_throw never returns.
	}
	else
	{
		// scm_misc_error(fe->get_name(), "unknown C++ exception", SCM_EOL);
		scm_error_scm(
			scm_from_utf8_symbol("C++ exception"),
			scm_from_utf8_string(func),
			scm_from_utf8_string("unknown C++ exception"),
			SCM_EOL,
			SCM_EOL);
		logger().error("Guile caught unknown C++ exception");
	}
}

/* ============================================================== */

#ifdef HAVE_GUILE2
 #define C(X) ((scm_t_subr) X)
#else
 #define C(X) ((SCM (*) ()) X)
#endif

void SchemeSmob::register_procs()
{
	register_proc("cog-atom",              1, 0, 0, C(ss_atom));
	register_proc("cog-handle",            1, 0, 0, C(ss_handle));
	register_proc("cog-undefined-handle",  0, 0, 0, C(ss_undefined_handle));
	register_proc("cog-new-node",          2, 0, 1, C(ss_new_node));
	register_proc("cog-new-link",          1, 0, 1, C(ss_new_link));
	register_proc("cog-node",              2, 0, 1, C(ss_node));
	register_proc("cog-link",              1, 0, 1, C(ss_link));
	register_proc("cog-delete",            1, 0, 1, C(ss_delete));
	register_proc("cog-delete-recursive",  1, 0, 1, C(ss_delete_recursive));
	register_proc("cog-purge",             1, 0, 1, C(ss_purge));
	register_proc("cog-purge-recursive",   1, 0, 1, C(ss_purge_recursive));
	register_proc("cog-atom?",             1, 0, 1, C(ss_atom_p));
	register_proc("cog-node?",             1, 0, 1, C(ss_node_p));
	register_proc("cog-link?",             1, 0, 1, C(ss_link_p));

	// property setters on atoms
	register_proc("cog-set-av!",           2, 0, 0, C(ss_set_av));
	register_proc("cog-set-tv!",           2, 0, 0, C(ss_set_tv));
	register_proc("cog-inc-vlti!",         1, 0, 0, C(ss_inc_vlti));
	register_proc("cog-dec-vlti!",         1, 0, 0, C(ss_dec_vlti));

	// property getters on atoms
	register_proc("cog-name",              1, 0, 0, C(ss_name));
	register_proc("cog-type",              1, 0, 0, C(ss_type));
	register_proc("cog-arity",             1, 0, 0, C(ss_arity));
	register_proc("cog-incoming-set",      1, 0, 0, C(ss_incoming_set));
	register_proc("cog-outgoing-set",      1, 0, 0, C(ss_outgoing_set));
	register_proc("cog-tv",                1, 0, 0, C(ss_tv));
	register_proc("cog-av",                1, 0, 0, C(ss_av));

	// Truth-values
	register_proc("cog-new-stv",           2, 0, 0, C(ss_new_stv));
	register_proc("cog-new-ctv",           3, 0, 0, C(ss_new_ctv));
	register_proc("cog-new-itv",           3, 0, 0, C(ss_new_itv));
	register_proc("cog-tv?",               1, 0, 0, C(ss_tv_p));
	register_proc("cog-stv?",              1, 0, 0, C(ss_stv_p));
	register_proc("cog-ctv?",              1, 0, 0, C(ss_ctv_p));
	register_proc("cog-itv?",              1, 0, 0, C(ss_itv_p));
	register_proc("cog-tv->alist",         1, 0, 0, C(ss_tv_get_value));

	// Atom Spaces
	register_proc("cog-new-atomspace",     0, 1, 0, C(ss_new_as));
	register_proc("cog-atomspace?",        1, 0, 0, C(ss_as_p));
	register_proc("cog-atomspace",         0, 0, 0, C(ss_get_as));
	register_proc("cog-set-atomspace!",    1, 0, 0, C(ss_set_as));

	// Attention values
	register_proc("cog-new-av",            3, 0, 0, C(ss_new_av));
	register_proc("cog-av?",               1, 0, 0, C(ss_av_p));
	register_proc("cog-av->alist",         1, 0, 0, C(ss_av_get_value));

	// AttentionalFocus
	register_proc("cog-af-boundary",       0, 0, 0, C(ss_af_boundary));
	register_proc("cog-set-af-boundary!",  1, 0, 0, C(ss_set_af_boundary));
	register_proc("cog-af",                0, 0, 0, C(ss_af));
    
	// ExecutionOutputLinks
	register_proc("cog-execute!",          1, 0, 0, C(ss_execute));

	// Atom types
	register_proc("cog-get-types",         0, 0, 0, C(ss_get_types));
	register_proc("cog-type->int",         1, 0, 0, C(ss_get_type));
	register_proc("cog-type?",             1, 0, 0, C(ss_type_p));
	register_proc("cog-node-type?",        1, 0, 0, C(ss_node_type_p));
	register_proc("cog-link-type?",        1, 0, 0, C(ss_link_type_p));
	register_proc("cog-get-subtypes",      1, 0, 0, C(ss_get_subtypes));
	register_proc("cog-subtype?",          2, 0, 0, C(ss_subtype_p));

	// Iterators
	register_proc("cog-map-type",          2, 0, 0, C(ss_map_type));
}

void SchemeSmob::register_proc(const char* name, int req, int opt, int rst, scm_t_subr fcn)
{
	scm_c_define_gsubr(name, req, opt, rst, fcn);
	scm_c_export(name, NULL);
}

#endif
/* ===================== END OF FILE ============================ */
