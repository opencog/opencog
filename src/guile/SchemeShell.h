/*
 * SchemeShell.h
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_SHELL_H
#define OPENCOG_SCHEME_SHELL_H
#ifdef HAVE_GUILE

#include <string>
#include <libguile.h>

namespace opencog {

class SchemeShell
{
	private:
		static bool is_inited;
		void register_procs(void);

		// Error handling stuff
		SCM error_string_port;
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM catch_handler(SCM, SCM);
		bool caught_error;

		// printfing of basic types
		std::string prt(SCM);

		// smobs
		struct cog;
		static scm_t_bits cog_tag;
		void init_smob_type(void);
		static SCM mark_cog(SCM);
		static size_t free_cog(SCM);
		static int print_cog(SCM, SCM, scm_print_state *);
		static SCM equalp_cog(SCM, SCM);

	public:
		SchemeShell(void);
		std::string eval(const std::string &);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SHELL_H */
