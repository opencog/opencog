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
		std::string eval(const std::string &);
};

}

#endif /* OPENCOG_SCHEME_SHELL_H */
