/*
 * opencog/atoms/reduct/PlusLink.cc
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Plus Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Plus Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atoms/NumberNode.h>
#include "PlusLink.h"

using namespace opencog;

PlusLink::PlusLink(const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(PLUS_LINK, oset, tv, av)
{
	init();
}

PlusLink::PlusLink(Type t, const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(t, oset, tv, av)
{
	if (not classserver().isA(t, PLUS_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a PlusLink");
	init();
}

PlusLink::PlusLink(Type t, const Handle& a, const Handle& b,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(t, a, b, tv, av)
{
	if (not classserver().isA(t, PLUS_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a PlusLink");
	init();
}

PlusLink::PlusLink(Link& l)
    : FreeLink(l)
{
	Type tscope = l.getType();
	if (not classserver().isA(tscope, PLUS_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a PlusLink");
	init();
}

void PlusLink::init(void)
{
}

/// reduce() -- reduce the expression by summing constants, etc.
///
/// No actual black-box evaluation or execution is performed. Only
/// clearbox reductions are performed.
///
/// Examples: the reduct of (PlusLink (NumberNode 2) (NumberNode 2))
/// is (NumberNode 4) -- its just a constant.
///
/// The reduct of (PlusLink (VariableNode "$x") (NumberNode 0)) is
/// (VariableNode "$x"), because adding zero to anything yeilds the
/// thing itself.
Handle PlusLink::reduce(void)
{
	// If the expression contains no free variables, and only constants,
	// then we should be able to reduce it to a NumberNode. Exceptions
	// are thrown if the expression is not summable.
	if (0 == _free_vars.size())
	{
		double sum = 0.0;
		for (const Handle& h: _outgoing)
		{
			Type t = h->getType();
			if (NUMBER_NODE != t and FREE_LINK != t)
				throw RuntimeException(TRACE_INFO,
					"Don't know how to reduce %s", h->toShortString().c_str());

			Handle redh(h);
			if (FREE_LINK == t)
			{
				FreeLinkPtr fff(FreeLinkCast(h));
				if (NULL == fff)
					fff = createFreeLink(*LinkCast(h));

				redh = fff->reduce();
			}

			NumberNodePtr nnn(NumberNodeCast(redh));
			if (NULL == nnn)
				nnn = createNumberNode(*NodeCast(redh));
			sum += nnn->getValue();
		}
		Handle result(createNumberNode(sum));

		// Place the result into the same atomspace we are in.
		if (_atomTable)
		{
			AtomSpace* as = _atomTable->getAtomSpace();
			return as->addAtom(result);
		}

		return result;
	}

	// If we are here, then there are variables.
	HandleSeq reduct;
	bool did_reduce = false;
	for (const Handle& h: _outgoing)
	{
		Type t = h->getType();
		if (NUMBER_NODE != t and
		    FREE_LINK != t and
		    VARIABLE_NODE != t)
			throw RuntimeException(TRACE_INFO,
				"Don't know how to reduce %s", h->toShortString().c_str());

		Handle redh(h);
		if (FREE_LINK == t)
		{
			FreeLinkPtr fff(FreeLinkCast(h));
			if (NULL == fff)
				fff = createFreeLink(*LinkCast(h));

			redh = fff->reduce();
		}

		if (h != redh) did_reduce = true;

		if (NUMBER_NODE == t)
		{
			NumberNodePtr nnn(NumberNodeCast(redh));
			if (NULL == nnn)
				nnn = createNumberNode(*NodeCast(redh));
			if (0.0 == nnn->getValue())
			{
				did_reduce = true;
				continue;
			}
		}
		reduct.push_back(redh);
	}

	if (not did_reduce) return getHandle();
	if (1 == reduct.size()) return reduct[0];

	Handle result(createLink(PLUS_LINK, reduct));

	// Place the result into the same atomspace we are in.
	if (_atomTable)
	{
		AtomSpace* as = _atomTable->getAtomSpace();
		return as->addAtom(result);
	}

	return result;
}
