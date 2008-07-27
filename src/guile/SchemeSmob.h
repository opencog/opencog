/*
 * SchemeSmob.h
 *
 * Guile SMOB interface for opencog atoms and truth values. 
 * This class implements the actual functions the guile shell invokes
 * when it sees the opencog-specific proceedures.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_SMOB_H
#define OPENCOG_SCHEME_SMOB_H
#ifdef HAVE_GUILE

#include <string>
#include <libguile.h>

namespace opencog {

class SchemeSmob
{
	friend class SchemeShell;

	private:
		static bool is_inited;
		void register_procs(void);

		// The handle tag is for opencog handles, only.
		static scm_t_bits cog_handle_tag;

		// The cog_misc_tag are for all other opencog types, such
		// as truth values, which are ephemeral (garbage-collected)
		static scm_t_bits cog_misc_tag;

		// Initializatino functions
		void init_smob_type(void);
		static int print_atom(SCM, SCM, scm_print_state *);
		static SCM equalp_atom(SCM, SCM);
		static SCM mark_misc(SCM);
		static size_t free_misc(SCM);

		// Atom creation and deletion functions
		static SCM ss_new_node(SCM, SCM, SCM);
		static SCM ss_new_link(SCM, SCM);
		static SCM ss_atom(SCM);
		static SCM ss_handle(SCM);
		static SCM ss_incoming_set(SCM);
		static SCM ss_outgoing_set(SCM);
		static SCM ss_delete(SCM);
		static SCM ss_delete_recursive(SCM);

		// Truth values
		static SCM ss_new_stv(SCM, SCM);

		// Misc utilities
		static std::string to_string(SCM);

	public:
		SchemeSmob(void);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SMOB_H */
