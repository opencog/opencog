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

Handle Instantiator::execution_output_link(const HandleSeq& oset)
{
	// This throws if it can't figure out the schema ...
	// should we try and catch here ?
	return ExecutionOutputLink::do_execute(_as, oset);

	// Unkown proceedure type.  Return it, maybe some other
	// execution-link handler will be able to process it.
	// return as->addLink(EXECUTION_OUTPUT_LINK, oset, TruthValue::TRUE_TV());
}

Handle Instantiator::walk_tree(Handle expr)
{
	std::cout << "walk_tree: expr = " << expr->toString() << std::endl;

	Type t = expr->getType();
	NodePtr nexpr(NodeCast(expr));
	if (nexpr)
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
	LinkPtr lexpr(LinkCast(expr));
	HandleSeq oset_results;
	for (Handle h : lexpr->getOutgoingSet())
		oset_results.push_back(walk_tree(h));

	// Fire execution links, if found.
	_did_exec = false;  // set flag on top-level only
	if (t == EXECUTION_OUTPUT_LINK)
	{
		_did_exec = true;
		return execution_output_link(oset_results);
	}

	// Now create a duplicate link, but with an outgoing set where
	// the variables have been substituted by their values.
	TruthValuePtr tv(expr->getTruthValue());
	return _as->addLink(t, oset_results, tv);
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
	_did_exec = false;

	Handle result = walk_tree(expr);
	if ((false == _did_exec) && (result == Handle::UNDEFINED))
		throw InvalidParamException(TRACE_INFO,
			"Failure to ground expression\n"
			"Ungrounded expr is %s\n",
			expr->toShortString().c_str());

	return result;
}

/* ===================== END OF FILE ===================== */
