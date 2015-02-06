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
	if (t == EXECUTION_OUTPUT_LINK)
	{
		// This throws if it can't figure out the schema ...
		// Let the throw pass right on up the stack.
		return ExecutionOutputLink::do_execute(_as, oset_results);
	}

	// Now create a duplicate link, but with an outgoing set where
	// the variables have been substituted by their values.
// #define FAST_VERSION 1 // XXX see the note below.
#ifdef FAST_VERSION
	return Handle(createLink(t, oset_results, expr->getTruthValue()));
#else
	return _as->addLink(t, oset_results, expr->getTruthValue());
#endif
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

// XXX FIXME -- there is a bug somewhere.  The FAST_VERSION should behave
// exactly the same way as the slow version, except that it should be
// faster.  It should be faster by avoiding excess atomspace calls.
// However, the fast version causes EinsteinUTest to fail.  I can't tell
// why. I conclude:
//
// 1) Either the AtomTable add() is buggy
// 2) Pattern matcher is buggy
// 3) Something is goofy about EinsteinUTest
// 4) Handles are misbehaving.
//
// Outcomes 1&2&4 are very disturbing, so this needs looking at.
// While trying to debug, it seems that the pattern matcher traverses
// atoms in a different order  for the fast and slow versions.  This
// shouldn't happen. The only way this might happen is if the pattern
// matcher is hitting the temporary atoms being made in the fast path.
// But this should not be possible, because in single-threaded mode,
// these temporary atoms should disappear when this method returns
// (since thier use count should drop to zero.)  So maybe thier use
// count isn't droipping? Very unlikely? So what explains the
// inconsistent pattern matcher behavior?
//
// Perhaps AtomTable::add() is mis-handling incoming set pointers,
// so that the temporary atoms remain in incoming sets. But even if
// that happened, the incoming pointers are weak, and would be pointing
// at the now-destroyed temporaries. And so would be discard when
// made strong ...
#ifndef FAST_VERSION
	return walk_tree(expr);
#else

	// The returned handle is not yet in the atomspace. Add it now.
	// We do this here, instead of in walk_tree(), because adding
	// atoms to the atomspace is an expensive process.  We can save
	// some time by doing it just once, right here, in one big batch.
	return _as->addAtom(walk_tree(expr));
#endif
}

/* ===================== END OF FILE ===================== */
