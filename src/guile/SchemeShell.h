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

		SCM string_outport;
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM catch_handler(SCM, SCM);
		bool caught_error;

		std::string prt(SCM);

	public:
		SchemeShell(void);
		std::string eval(const std::string &);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SHELL_H */
