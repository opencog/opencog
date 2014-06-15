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

#include <opencog/execution/ExecutionLink.h>

#include "Instantiator.h"

using namespace opencog;

Handle Instantiator::execution_link()
{
	// This throws if it can't figure out the schema ...
	// should we try and catch here ?
	return ExecutionLink::do_execute(_as, _oset);

	// Unkown proceedure type.  Return it, maybe some other
	// execution-link handler will be able to process it.
	// return as->addLink(EXECUTION_LINK, oset, TruthValue::TRUE_TV());
}

bool Instantiator::walk_tree(Handle expr)
{
	Type t = expr->getType();
	NodePtr nexpr(NodeCast(expr));
	if (nexpr)
	{
		if (VARIABLE_NODE != t) {
			_oset.push_back(expr);
			return false;
		}

		// If we are here, we found a variable. Look it up.
		std::map<Handle,Handle>::const_iterator it = _vmap->find(expr);
		if (_vmap->end() != it) {
			Handle soln = it->second;
			_oset.push_back(soln);
		} else {
			_oset.push_back(expr);
		}
		return false;
	}

	// If we are here, then we have a link. Walk it.
	std::vector<Handle> save_oset = _oset;
	_oset.clear();

	// Walk the subtree, substituting values for variables.
	LinkPtr lexpr(LinkCast(expr));
	for (Handle h : lexpr->getOutgoingSet())
		walk_tree(h);

	// Fire execution links, if found.
	did_exec = false;  // set flag on top-level only
	if (t == EXECUTION_LINK)
	{
		did_exec = true;
		Handle sh(execution_link());
		_oset = save_oset;
		if (Handle::UNDEFINED != sh)
		{
			_oset.push_back(sh);
		}
		return false;
	}

	// Now create a duplicate link, but with an outgoing set where
	// the variables have been substituted by their values.
	TruthValuePtr tv(expr->getTruthValue());
	Handle sh(_as->addLink(t, _oset, tv));

	_oset = save_oset;
	_oset.push_back(sh);

	return false;
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
Handle Instantiator::instantiate(Handle& expr, const std::map<Handle, Handle> &vars)
	throw (InvalidParamException)
{
	// throw, not assert, because this is a user error ...
	if (Handle::UNDEFINED == expr)
		throw InvalidParamException(TRACE_INFO,
			"Asked to ground a null expression");

	_vmap = &vars;
	_oset.clear();
	did_exec = false;

	walk_tree(expr);
	if ((false == did_exec) && (_oset.size() != 1))
		throw InvalidParamException(TRACE_INFO,
			"Failure to ground expression (found %d groundings)\n"
			"Ungrounded expr is %s\n",
			_oset.size(), expr->toShortString().c_str());

	if (_oset.size() >= 1)
		return _oset[0];
	return Handle::UNDEFINED;
}

/* ===================== END OF FILE ===================== */
