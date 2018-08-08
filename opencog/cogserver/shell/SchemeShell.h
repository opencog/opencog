/*
 * SchemeShell.h
 *
 * Simple scheme shell
 * Copyright (c) 2008, 2013 Linas Vepstas <linas@linas.org>
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

#include <opencog/cogserver/shell/GenericShell.h>

namespace opencog {
/** \addtogroup grp_server
 *  @{
 */

class SchemeShell : public GenericShell
{
	protected:
		void thread_init();
		static std::string _prompt;

	public:
		SchemeShell(void);
		virtual ~SchemeShell();
		virtual GenericEval* get_evaluator(void);
};

/** @}*/
}

#endif // _OPENCOG_SCHEME_SHELL_H

#endif // HAVE_GUILE
