/*
 * opencog/atoms/reduct/FoldLink.cc
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Fold Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Fold Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <limits>

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atoms/NumberNode.h>
#include "FoldLink.h"

using namespace opencog;

FoldLink::FoldLink(const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(FOLD_LINK, oset, tv, av)
{
	init();
}

FoldLink::FoldLink(Type t, const HandleSeq& oset,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(t, oset, tv, av)
{
	if (not classserver().isA(t, FOLD_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FoldLink");
	init();
}

FoldLink::FoldLink(Type t, const Handle& a, const Handle& b,
                   TruthValuePtr tv,
                   AttentionValuePtr av)
    : FreeLink(t, a, b, tv, av)
{
	if (not classserver().isA(t, FOLD_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FoldLink");
	init();
}

FoldLink::FoldLink(Link& l)
    : FreeLink(l)
{
	Type tscope = l.getType();
	if (not classserver().isA(tscope, FOLD_LINK))
		throw InvalidParamException(TRACE_INFO, "Expecting a FoldLink");
	init();
}

void FoldLink::init(void)
{
	knil = std::numeric_limits<double>::quiet_NaN();
	kons = NULL;
}

/// reduce() -- reduce the expression by summing constants, etc.
///
/// No actual black-box evaluation or execution is performed. Only
/// clearbox reductions are performed.
///
/// Examples: the reduct of (FoldLink (NumberNode 2) (NumberNode 2))
/// is (NumberNode 4) -- its just a constant.
///
/// The reduct of (FoldLink (VariableNode "$x") (NumberNode 0)) is
/// (VariableNode "$x"), because adding zero to anything yeilds the
/// thing itself.
Handle FoldLink::reduce(void)
{
	// Assume that the expression is a mixture of constants and variables.
	// Sum the constants, and eliminate the nils.
	HandleSeq reduct;
	bool did_reduce = false;
	double sum = knil;
	for (const Handle& h: _outgoing)
	{
		Type t = h->getType();
		if (NUMBER_NODE != t and
		    VARIABLE_NODE != t and
		    (not classserver().isA(t, FREE_LINK))
		)
			throw RuntimeException(TRACE_INFO,
				"Don't know how to reduce %s", h->toShortString().c_str());

		Handle redh(h);
		if (classserver().isA(t, FREE_LINK))
		{
			FreeLinkPtr fff(FreeLinkCast(h));
			if (NULL == fff)
				fff = createFreeLink(*LinkCast(h));

			redh = fff->reduce();
			t = redh->getType();
		}

		if (h != redh) did_reduce = true;

		if (NUMBER_NODE == t)
		{
			NumberNodePtr nnn(NumberNodeCast(redh));
			if (NULL == nnn)
				nnn = createNumberNode(*NodeCast(redh));
			sum = kons(sum, nnn->getValue());
			did_reduce = true;
			continue;
		}
		reduct.push_back(redh);
	}

	// If nothing reduced, nothing to do.
	if (not did_reduce) return getHandle();

	// If it reduced to just one number:
	if (0 == reduct.size())
		return Handle(createNumberNode(sum));

	// If it didn't sum to nil, then we have to keep it.
	if (knil != sum)
		reduct.push_back(Handle(createNumberNode(sum)));

	// If it reduced to just one thing:	
	if (1 == reduct.size()) return reduct[0];

	Handle result(createLink(getType(), reduct));

	// Place the result into the same atomspace we are in.
	if (_atomTable)
	{
		AtomSpace* as = _atomTable->getAtomSpace();
		return as->addAtom(result);
	}

	return result;
}
