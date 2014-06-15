/*
 * Instantiator.h
 *
 * Copyright (C) 2009, 2014 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
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

#ifndef _OPENCOG_INSTANTIATOR_H
#define _OPENCOG_INSTANTIATOR_H

#include <vector>

#include <opencog/atomspace/AtomSpace.h>

/**
 * class Instantiator -- create grounded expressions from ungrounded ones.
 * Given an ungrounded expression (i.e. an expression containing variables)
 * and a map between variables and ground terms, it will create a new
 * expression, with the ground terms substituted for the variables.
 */
namespace opencog {

class Instantiator
{
	private:
		AtomSpace *_as;
		const std::map<Handle, Handle> *_vmap;

		std::vector<Handle> _oset;
		bool walk_tree(Handle tree);
		Handle execution_link(void);
		bool did_exec;

	public:
		Instantiator(AtomSpace* as) : _as(as) {}

		Handle instantiate(Handle& expr, const std::map<Handle, Handle> &vars)
			throw (InvalidParamException);
};

} // namespace opencog

#endif // _OPENCOG_INSTANTIATOR_H

