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
#include "SchemeEval.h"

namespace opencog {

class SchemeSmob;

class SchemeShell
{
	private:
		SchemeEval evaluator;

		std::string normal_prompt;
		std::string pending_prompt;
		std::string input_line;
		bool pending_input;
		bool show_output;

		// Error handling stuff
		SCM error_string_port;
		SCM captured_stack;
		static SCM preunwind_handler_wrapper(void *, SCM, SCM);
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM preunwind_handler(SCM, SCM);
		SCM catch_handler(SCM, SCM);
		bool caught_error;

		// output port
		SCM outport;

		SchemeSmob *funcs;

	public:
		SchemeShell(void);
		void hush_output(bool);
		std::string eval(const std::string &);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SHELL_H */
