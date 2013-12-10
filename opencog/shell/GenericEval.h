/*
 * GenericEval.h
 *
 * Template for a generic evaluator
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

#ifndef _OPENCOG_GENERIC_EVAL_H
#define _OPENCOG_GENERIC_EVAL_H

#include <string>

/**
 * The GenericEval class implements an very simple API for language
 * evaluators.  Currently, its sole purpose in life is to make the
 * abstraction of the GenericShell class possible.
 */

namespace opencog {
/** \addtogroup grp_server
 *  @{
 */

class GenericEval
{
	protected:
		bool _pending_input;

	public:
		GenericEval(void) : _pending_input(false) {}
		virtual ~GenericEval() {}

		/**
		 * Return true if the expression was incomplete, and more is expected
		 * (for example, more closing parens are expected)
		 */
		virtual bool input_pending()
		{
			return _pending_input;
		}

		/**
		 * Clear the error state, the input buffers, etc.
		 */
		virtual void clear_pending()
		{
			_pending_input = false;
		}

		virtual std::string eval(const std::string&) = 0;
};

/** @}*/
}

#endif // _OPENCOG_GENERIC_EVAL_H
