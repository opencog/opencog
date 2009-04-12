/*
 * SchemeSmob.h
 *
 * Guile SMOB interface for opencog atoms and truth values. 
 * This class implements the actual functions the guile shell invokes
 * when it sees the opencog-specific proceedures.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#ifdef HAVE_GUILE

#ifndef _OPENCOG_SCHEME_SMOB_H
#define _OPENCOG_SCHEME_SMOB_H

#include <string>
#include <vector>

#include <libguile.h>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/VersionHandle.h>

namespace opencog {

class Atom;

class SchemeSmob
{
	friend class SchemeEval;

	private:

		enum
		{
			COG_HANDLE = 1,
			COG_TV,  // truth values
			COG_VH,  // version handles
		};

		static bool is_inited;
		static void register_procs(void);

		// The handle tag is for opencog handles, only.
		static scm_t_bits cog_handle_tag;

		// The cog_misc_tag are for all other opencog types, such
		// as truth values, which are ephemeral (garbage-collected)
		static scm_t_bits cog_misc_tag;

		// Initialization functions
		static void init_smob_type(void);

		static int print_atom(SCM, SCM, scm_print_state *);
		static SCM equalp_atom(SCM, SCM);
		static size_t free_atom(SCM);
		static SCM mark_misc(SCM);
		static size_t free_misc(SCM);

		// Atom creation and deletion functions
		static SCM handle_to_scm (Handle);
		static SCM ss_new_node(SCM, SCM, SCM);
		static SCM ss_new_link(SCM, SCM);
		static SCM ss_node(SCM, SCM, SCM);
		static SCM ss_link(SCM, SCM);
		static SCM ss_delete(SCM);
		static SCM ss_delete_recursive(SCM);
		static SCM ss_atom_p(SCM);

		// Atoms to ints, and back.
		static SCM ss_atom(SCM);
		static SCM ss_handle(SCM);

		// Atom properties
		static SCM ss_name(SCM);
		static SCM ss_type(SCM);
		static SCM ss_arity(SCM);
		static SCM ss_tv(SCM);
		static SCM ss_incoming_set(SCM);
		static SCM ss_outgoing_set(SCM);

		// AtomTable query functions
		static SCM ss_map_type(SCM, SCM);
		static SCM ss_get_types(void);

		// Truth values
		static SCM ss_new_stv(SCM, SCM);
		static SCM ss_new_ctv(SCM, SCM, SCM);
		static SCM ss_new_itv(SCM, SCM, SCM);
		static SCM ss_tv_p(SCM);
		static SCM take_tv(TruthValue *);
		static SCM ss_tv_get_value(SCM);

		// Version handles
		static SCM ss_new_vh(SCM, SCM);
		static SCM ss_vh_p(SCM);
		static SCM take_vh(VersionHandle *);
		static SCM ss_vh_get_value(SCM);

		// Callback into misc C++ code.
		static SCM ss_ad_hoc(SCM, SCM);
		static SCM pln_bc(SCM, SCM);

		// Misc utilities
		static std::string decode_string (SCM, const char *, const char *);
		static std::string to_string(SCM);
		static std::string handle_to_string(SCM);
		static std::string handle_to_string(Handle, int);
		static std::string misc_to_string(SCM);
		static std::string tv_to_string(const TruthValue *);
		static std::string vh_to_string(const VersionHandle *);
		static TruthValue *get_tv_from_list(SCM);

		static Type validate_atom(SCM, const char *);
		static Type validate_node(SCM, const char *);
		static const Atom * verify_atom(SCM, const char *);
		static Handle verify_handle(SCM, const char *);
		static std::vector<Handle> decode_handle_list (SCM, const char *);

	public:
		static void init(void);
		SchemeSmob(void);
};

} // namespace opencog

#endif // _OPENCOG_SCHEME_SMOB_H

#endif // HAVE_GUILE
