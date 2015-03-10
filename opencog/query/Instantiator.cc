/*
 * Instantiator.cc
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

#include <opencog/execution/ExecutionOutputLink.h>

#include "Instantiator.h"

using namespace opencog;

Handle Instantiator::walk_tree(Handle expr)
{
	Type t = expr->getType();
	LinkPtr lexpr(LinkCast(expr));
	if (not lexpr)
	{
		if (VARIABLE_NODE != t)
			return expr;

		// If we are here, we found a variable. Look it up. Return a
		// grounding if it has one, otherwise return the variable
		// itself
		std::map<Handle,Handle>::const_iterator it = _vmap->find(expr);
		return _vmap->end() != it ? it->second : expr;
	}

	// If we are here, then we have a link. Walk it.

	// Walk the subtree, substituting values for variables.
	HandleSeq oset_results;
	for (Handle h : lexpr->getOutgoingSet())
		oset_results.push_back(walk_tree(h));

	// Fire execution links, if found.
	if ((EXECUTION_OUTPUT_LINK == t)
	   or (PLUS_LINK == t)
	   or (TIMES_LINK == t))
	{
		// The atoms being created above might not all be in the
		// atomspace, just yet. Because we have no clue what the
		// ExecutionOutputLink might do, we had best put them
		// there now. Just as well, because it seems the scheme
		// (and python) bindings get tripped up by the UUID==-1
		// of uninserted atoms.
		size_t sz = oset_results.size();
		for (size_t i=0; i< sz; i++)
			oset_results[i] = _as->addAtom(oset_results[i]);

		// This throws if it can't figure out the schema ...
		// Let the throw pass right on up the stack.
		return ExecutionOutputLink::do_execute(_as, t, oset_results);
	}

	// Now create a duplicate link, but with an outgoing set where
	// the variables have been substituted by their values.
	return Handle(createLink(t, oset_results, expr->getTruthValue()));
}

/**
 * instantiate -- create a grounded expression from an ungrounded one.
 *
 * Given a handle to an ungrounded expression, and a set of groundings,
 * this will create a grounded expression.
 *
 * The set of groundings is to be passed in with the map 'vars', which
 * maps variable names to their groundings -- it maps variable names to
 * atoms that already exist in the atomspace.  This method will then go
 * through all of the variables in the expression, and substitute them
 * with their values, creating a new expression. The new expression is
 * added to the atomspace, and its handle is returned.
 */
Handle Instantiator::instantiate(Handle& expr,
                                 const std::map<Handle, Handle> &vars)
	throw (InvalidParamException)
{
	// throw, not assert, because this is a user error ...
	if (Handle::UNDEFINED == expr)
		throw InvalidParamException(TRACE_INFO,
			"Asked to ground a null expression");

	_vmap = &vars;

	// The returned handle is not yet in the atomspace. Add it now.
	// We do this here, instead of in walk_tree(), because adding
	// atoms to the atomspace is an expensive process.  We can save
	// some time by doing it just once, right here, in one big batch.
	return _as->addAtom(walk_tree(expr));
}

/* ===================== END OF FILE ===================== */
