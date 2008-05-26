/*
 * SchemeSmob.h
 *
 * Simple scheme shell
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

		static scm_t_bits cog_tag;
		void init_smob_type(void);
		static int print_cog(SCM, SCM, scm_print_state *);
		static SCM equalp_cog(SCM, SCM);

		static std::string to_string(SCM);

		// Functions
		static SCM ss_new_node(SCM, SCM);
		static SCM ss_new_link(SCM, SCM);
		static SCM ss_atom(SCM);
		static SCM ss_handle(SCM);

	public:
		SchemeSmob(void);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SMOB_H */
