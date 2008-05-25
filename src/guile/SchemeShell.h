/*
 * SchemeShell.h
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_SHELL_H
#define OPENCOG_SCHEME_SHELL_H

#include <string>

namespace opencog {

class SchemeShell
{
	private:
		static bool is_inited;
		void register_procs(void);

	public:
		SchemeShell(void);
		void eval(const char *);
};

}

#endif /* OPENCOG_SCHEME_SHELL_H */
