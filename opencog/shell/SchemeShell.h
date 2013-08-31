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
#include <opencog/server/ConsoleSocket.h>

namespace opencog {
/** \addtogroup grp_server
 *  @{
 */

class SchemeShellModule;

class SchemeShell : public GenericShell
{
	friend class SchemeShellModule;
	private:
		SchemeEval *evaluator;

		std::string normal_prompt;
		std::string pending_prompt;
		std::string abort_prompt;
		const std::string& get_prompt(void);
		bool show_output;
		bool show_prompt;

		ConsoleSocket *socket;
		std::string do_eval(const std::string &);
		bool self_destruct;

	protected:
		void set_socket(ConsoleSocket *);

	public:
		SchemeShell(void);
		virtual ~SchemeShell();

		virtual void eval(const std::string &, ConsoleSocket *);
		virtual void socketClosed(void);

		void hush_output(bool);
		void hush_prompt(bool);
};

/** @}*/
}

#endif // _OPENCOG_SCHEME_SHELL_H

#endif // HAVE_GUILE
