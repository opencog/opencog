/*
 * SchemeShell.h
 *
 * Simple scheme shell
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

#ifndef _OPENCOG_SCHEME_SHELL_H
#define _OPENCOG_SCHEME_SHELL_H

#include <string>

#include <opencog/guile/SchemeEval.h>
#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

namespace opencog {

class SchemeShell : public GenericShell
{
	private:
		DECLARE_CMD_REQUEST(SchemeShell, "scm", shellout,
			"Enter the scheme shell",
			"Usage: scm\n\n"
			"Enter the scheme interpreter shell. This shell provides a rich\n"
			"and easy-to-use envirnoment for creating, deleting and manipulating\n"
			"OpenCog atoms and truth values. It provides a full R5RS-compliant\n"
			"interactive scheme shell, based on the GNU Guile extension language.")

		SchemeEval *evaluator;

		std::string normal_prompt;
		std::string pending_prompt;
		std::string abort_prompt;
		const std::string& get_prompt(void);
		bool show_output;

		ConsoleSocket *cs;
		std::string do_eval(const std::string &);

	public:
		SchemeShell(void);
		virtual ~SchemeShell();

		virtual const char *id(void);
		virtual void init(void);
		virtual void eval(const std::string &, ConsoleSocket *);

		void hush_output(bool);
};

}

#endif // _OPENCOG_SCHEME_SHELL_H

#endif // HAVE_GUILE
