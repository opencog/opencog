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
#include "GenericSocket.h"
#include "GenericModule.h"

namespace opencog {

class GenericShell : public Module
{
	friend class GenericModule;
	friend class GenericSocket;

	protected:
		std::string abort_prompt;
		std::string normal_prompt;
		std::string pending_prompt;

	public:
		GenericShell(void);
		virtual ~GenericShell() {}

		virtual const char *id(void) = 0;
		virtual void eval(const std::string &, GenericSocket&) = 0;
};

}

#endif // _OPENCOG_GENERIC_SHELL_H
