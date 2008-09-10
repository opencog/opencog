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

class SchemeSmob;

class SchemeShell
{
	private:
		static bool is_inited;

		std::string normal_prompt;
		std::string pending_prompt;
		std::string input_line;
		bool pending_input;
		bool show_output;

		// Error handling stuff
		SCM error_string_port;
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM catch_handler(SCM, SCM);
		bool caught_error;

		// printfing of basic types
		std::string prt(SCM);

		SchemeSmob *funcs;

	public:
		SchemeShell(void);
		void hush_output(bool);
		std::string eval(const std::string &);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SHELL_H */
