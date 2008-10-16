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
#include "SchemeEval.h"

namespace opencog {

class SchemeShell
{
	private:
		SchemeEval evaluator;

		std::string normal_prompt;
		std::string pending_prompt;
		bool show_output;

	public:
		SchemeShell(void);
		void hush_output(bool);
		std::string eval(const std::string &);
};

}

#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_SHELL_H */
