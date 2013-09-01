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

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/VersionHandle.h>

namespace opencog {
/** \addtogroup grp_smob
 *  @{
 */

class Atom;

class SchemeSmob
{
		friend class SchemeEval;
		friend class PrimitiveEnviron;
		template <typename TT> friend class SchemePrimitive;

	private:

		enum {
			COG_HANDLE = 1,
			COG_TV,     // truth values
			COG_VH,     // version handles
			COG_AV,     // attention values
			COG_EXTEND // callbacks into C++ code.
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
		static int print_misc(SCM, SCM, scm_print_state *);
		static SCM mark_misc(SCM);
		static size_t free_misc(SCM);

		// Atom creation and deletion functions
		static SCM ss_new_node(SCM, SCM, SCM);
		static SCM ss_new_link(SCM, SCM);
		static SCM ss_node(SCM, SCM, SCM);
		static SCM ss_link(SCM, SCM);
		static SCM ss_delete(SCM);
		static SCM ss_delete_recursive(SCM);
		static SCM ss_atom_p(SCM);
		static SCM ss_node_p(SCM);
		static SCM ss_link_p(SCM);

		// Atoms to ints, and back.
		static SCM ss_atom(SCM);
		static SCM ss_handle(SCM);


		// return the int of Handle::UNDEFINED
		static SCM ss_undefined_handle(void);

		// Set properties of atoms
		static SCM ss_set_av(SCM, SCM);
		static SCM ss_set_tv(SCM, SCM);
		static SCM ss_inc_vlti(SCM);
		static SCM ss_dec_vlti(SCM);

		// Atom properties
		static SCM ss_name(SCM);
		static SCM ss_type(SCM);
		static SCM ss_arity(SCM);
		static SCM ss_av(SCM);
		static SCM ss_tv(SCM);
		static SCM ss_incoming_set(SCM);
		static SCM ss_outgoing_set(SCM);

		// AtomSpace query functions
		static SCM ss_map_type(SCM, SCM);
		static SCM ss_get_types(void);
		static SCM ss_type_p(SCM);
		static SCM ss_get_subtypes(SCM);
		static SCM ss_subtype_p(SCM, SCM);

		// Truth values
		static SCM ss_new_stv(SCM, SCM);
		static SCM ss_new_ctv(SCM, SCM, SCM);
		static SCM ss_new_itv(SCM, SCM, SCM);
		static SCM ss_new_mtv(SCM, SCM);
		static SCM ss_set_vtv(SCM, SCM, SCM);
		static SCM ss_tv_p(SCM);
		static SCM tv_p(SCM, TruthValueType);
		static SCM ss_stv_p(SCM);
		static SCM ss_ctv_p(SCM);
		static SCM ss_itv_p(SCM);
		static SCM ss_mtv_p(SCM);
		static SCM take_tv(TruthValue *);
		static SCM ss_tv_get_value(SCM);

		// Version handles
		static SCM ss_new_vh(SCM, SCM);
		static SCM ss_vh_p(SCM);
		static SCM take_vh(VersionHandle *);
		static SCM ss_vh_get_value(SCM);

		// Attention values
		static SCM ss_new_av(SCM, SCM, SCM);
		static SCM ss_av_p(SCM);
		static SCM take_av(AttentionValue *);
		static SCM ss_av_get_value(SCM);

		// Callback into misc C++ code.
		static SCM ss_ad_hoc(SCM, SCM);
		static SCM pln_bc(SCM, SCM);

		// Misc utilities
		static std::string to_string(SCM);
		static std::string handle_to_string(SCM);
		static std::string handle_to_string(Handle, int);
		static std::string misc_to_string(SCM);
		static TruthValue *get_tv_from_list(SCM);
		static AttentionValue *get_av_from_list(SCM);

		// validate arguments coming from scheme passing into C++
		static Type verify_atom_type(SCM, const char *, int pos = 1);
		static Type verify_node_type(SCM, const char *, int pos = 1);
		static Handle verify_handle(SCM, const char *, int pos = 1);
		static VersionHandle * verify_vh(SCM, const char *, int pos = 1);
		static TruthValue * verify_tv(SCM, const char *, int pos = 1);
		static AttentionValue * verify_av(SCM, const char *, int pos = 1);
		static std::vector<Handle> verify_handle_list (SCM, const char *,
		                                  int pos = 1);
		static std::string verify_string (SCM, const char *, int pos = 1,
		                                  const char *msg = "expecting string");
		static int verify_int (SCM, const char *, int pos = 1,
		                       const char *msg = "expecting integer");

		static AtomSpace* atomspace;
		static void init(AtomSpace *as);
		SchemeSmob(AtomSpace *as);
	public:
		// Helper functions XXX why are these public ??
		// XXX Becuase the embodiment code uses them :-(
		// The embodiment code should be refactored to no use these.
		static SCM handle_to_scm(Handle);
		static Handle scm_to_handle(SCM);

		// Utility printing functions
		static std::string to_string(Handle);
		static std::string av_to_string(const AttentionValue *);
		static std::string tv_to_string(const TruthValue *);
		static std::string vh_to_string(const VersionHandle *);
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_SCHEME_SMOB_H

#endif // HAVE_GUILE
