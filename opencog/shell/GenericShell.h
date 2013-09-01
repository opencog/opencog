/*
 * GenericShell.h
 *
 * Template for a generic shell
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

#ifndef _OPENCOG_GENERIC_SHELL_H
#define _OPENCOG_GENERIC_SHELL_H

#include <string>

#include <opencog/server/Module.h>

/**
 * The GenericShell class implements an "escape" from the default cogserver
 * command processor. It is useful when a module has a large number of
 * commands, or a peculiar and complex syntax that must be handled. It
 * works by by-passing all of the command-processing apparatus in the
 * cogserver, and instead passing all socket I/O to the "eval" method
 * in this class. The eval method is then free to parse the input in
 * any way desired.
 *
 * If instead a module has only a small number of simple commands that
 * need to be implemented, then the DECLARE_CMD_REQUEST function, defined
 * in Request.h, provides a simpler and easier way implementing comands.
 */

namespace opencog {
/** \addtogroup grp_server
 *  @{
 */

class ConsoleSocket;

class GenericShell
{
	protected:
		std::string abort_prompt;
		std::string normal_prompt;
		std::string pending_prompt;

	public:
		GenericShell(void);
		virtual ~GenericShell() {}

		virtual void eval(const std::string &, ConsoleSocket *) = 0;
		virtual void socketClosed(void) = 0;
};

/** @}*/
}

#endif // _OPENCOG_GENERIC_SHELL_H
