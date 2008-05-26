/*
 * SchemeSmob.h
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_SMOB_H
#define OPENCOG_SCHEME_SMOB_H
#ifdef HAVE_GUILE

#include <libguile.h>

namespace opencog {

class SchemeSmob
{
	private:
		static bool is_inited;
		void register_procs(void);

		void init_smob_type(void);
		static SCM mark_cog(SCM);
		static size_t free_cog(SCM);
		static int print_cog(SCM, SCM, scm_print_state *);
		static SCM equalp_cog(SCM, SCM);

		// Functions
		static SCM ss_atom(SCM);

	public:
		SchemeSmob(void);
		static scm_t_bits cog_tag;
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SMOB_H */
