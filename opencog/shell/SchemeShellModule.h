/*
 * SchemeShellModule.h
 *
 * Module for starting up scheme shells
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifdef HAVE_GUILE

#ifndef _OPENCOG_SCHEME_SHELL_MODULE_H
#define _OPENCOG_SCHEME_SHELL_MODULE_H

#include <string>

#include <opencog/shell/SchemeShell.h>
#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

namespace opencog {

class SchemeShellModule : public Module
{
	private:
		DECLARE_CMD_REQUEST(SchemeShellModule, "scm", shellout,
			"Enter the scheme shell",
			"Usage: scm [hush|quiet]\n\n"
			"Enter the scheme interpreter shell. This shell provides a rich\n"
			"and easy-to-use environment for creating, deleting and manipulating\n"
			"OpenCog atoms and truth values. It provides a full R5RS-compliant\n"
			"interactive scheme shell, based on the GNU Guile extension language.\n\n"
			"If 'hush' or 'quiet' is specified after the command, then the prompt\n"
			"will not be returned.  This is nice when catting large scripts using\n"
			"netcat, as it avoids printing garbage when the scripts work well.\n",
			true, false)

		DECLARE_CMD_REQUEST(SchemeShellModule, "scm-eval", do_eval,
			"Run some scheme code",
			"Usage: scm-eval <scheme code>\n\n"
			"Evaluate the specified Scheme code. It does not need to be quoted.",
			false, false)

	public:
		SchemeShellModule(CogServer&);
		virtual ~SchemeShellModule();

		static const char *id(void);
		virtual void init(void);
};

}

#endif // _OPENCOG_SCHEME_SHELL_MODULE_H

#endif // HAVE_GUILE
