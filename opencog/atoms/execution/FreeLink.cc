/*
 * opencog/atoms/execution/FreeLink.cc
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
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

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/ClassServer.h>
#include "FreeLink.h"

using namespace opencog;

FreeLink::FreeLink(const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : Link(FREE_LINK, oset, tv, av)
{
	init();
}

FreeLink::FreeLink(Type t, const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : Link(t, oset, tv, av)
{
	if (not classserver().isA(t, FREE_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FreeLink");
	init();
}

FreeLink::FreeLink(Type t, const Handle& a, const Handle& b,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : Link(t, a, b, tv, av)
{
	if (not classserver().isA(t, FREE_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FreeLink");
	init();
}

FreeLink::FreeLink(Link& l)
    : Link(l)
{
	Type tscope = l.getType();
	if (not classserver().isA(tscope, FREE_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FreeLink");
	init();
}

/// Create an ordered set of the free variables in this link.
///
void FreeLink::find_vars(std::set<Handle>& varset, const HandleSeq& oset)
{
	for (const Handle& h : oset)
	{
		if (VARIABLE_NODE == h->getType() and
		    0 == varset.count(h))
		{
			_free_vars.push_back(h);
			varset.insert(h);
		}
		LinkPtr lll(LinkCast(h));
		if (NULL == lll) return;

		find_vars(varset, lll->getOutgoingSet());
	}
}

void FreeLink::init(void)
{
	std::set<Handle> varset;
	find_vars(varset, _outgoing);
}
