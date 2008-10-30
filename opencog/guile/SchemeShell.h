/*
 * SchemeShell.h
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#ifndef _OPENCOG_SCHEME_SHELL_H
#define _OPENCOG_SCHEME_SHELL_H

#include <string>

#include <libguile.h>
#include "SchemeEval.h"

#include <opencog/guile/SchemeSocket.h>

namespace opencog {

class SchemeSmob;

class SchemeShell
{
	friend class SchemeModule;
	friend class SchemeSocket;

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
		bool caught_error;

		// output port
		SCM outport;

		SchemeSmob *funcs;

	public:
		SchemeShell();
		~SchemeShell();
		void hush_output(bool);
		SCM preunwind_handler(SCM, SCM);
		SCM catch_handler(SCM, SCM);
		void eval(const std::string &, SchemeSocket&);
};

}

#endif // _OPENCOG_SCHEME_SHELL_H

#endif // HAVE_GUILE
