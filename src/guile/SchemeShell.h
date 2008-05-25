/*
 * SchemeShell.h
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_SHELL_H
#define OPENCOG_SCHEME_SHELL_H

#include <Atom.h>

namespace opencog {

class SchemeShell
{
	private:
		static bool is_inited = false;
		void register_procs(void);

	public:
		SchemeShell(void);
};

}

#endif /* OPENCOG_SCHEME_SHELL_H */
