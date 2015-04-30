/*
 * DeleteLink.cc
 *
 * Copyright (C) 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomutils/FindUtils.h>

#include "DeleteLink.h"

using namespace opencog;

void DeleteLink::init(const HandleSeq& oset)
{
	// The handleset must contain an un-qutoed VariableNode in it,
	// somewhere. If it doesn't, then the entire Handleset should be
	// deleted (removed from the atomspace).
	//
	// This removes from the atomspace, not the atomtable, so that the
	// the atom is deleted from the backingstore as well.  This requires
	// asking the AtomTable which AtomSpace it belongs to.

	bool valid = false;
	for (const Handle& h : oset)
	{
		if (contains_atomtype(h, VARIABLE_NODE)) valid = true;
	}

	if (not valid)
	{
		for (Handle h : oset)
		{
			AtomTable * at = h->getAtomTable();
			if (NULL == at) continue;
			AtomSpace * as = at->getAtomSpace();
			if (NULL == as)
				at->extract(h, true);
			else
				as->removeAtom(h, true);
		}
	}

	// It can only ever hold VariableNodes!
	if (not valid)
		// throw DeleteException();
		throw InvalidParamException(TRACE_INFO,
			"Cannot insert a fully grounded DeleteLink into the atomsapce!");
} 

void DeleteLink::del(AtomSpace * as, const HandleSeq& oset) const
{
	for (const Handle& h : oset)
	{
		Type t = h->getType();
		if (VARIABLE_NODE != t)
			as->removeAtom(h, true);
	}
}

void DeleteLink::del(AtomSpace * as) const
{
	del(as, _outgoing);
}

DeleteLink::DeleteLink(const HandleSeq& oset,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(DEFINE_LINK, oset, tv, av)
{
	init(oset);
}

DeleteLink::DeleteLink(const Handle& name, const Handle& defn,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(DEFINE_LINK, HandleSeq({name, defn}), tv, av)
{
	init(getOutgoingSet());
}

DeleteLink::DeleteLink(Link &l)
	: Link(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, DELETE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a DeleteLink, got %s", tname.c_str());
	}

	init(l.getOutgoingSet());
}

/* ===================== END OF FILE ===================== */
