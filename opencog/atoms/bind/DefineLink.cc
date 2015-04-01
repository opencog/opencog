/*
 * DefineLink.cc
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
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
#include <opencog/atoms/TypeNode.h>

#include "DefineLink.h"

using namespace opencog;

void DefineLink::init(const HandleSeq& oset)
{
	// Must have name and body
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"Expecting name and definition, got size %d", oset.size());

	// The name mut not have been previously defined before.
	HandleSeq ename;
   oset[0]->getIncomingSetByType(std::back_inserter(ename), DEFINE);
	if (0 < ename.size())
		throw InvalidParamException(TRACE_INFO,
			"This is already defined; remoce before redfining!");

	Type t = oset[1]->getType();
	if (not classserver().isA(t, LAMBDA_LINK))
	{
		const std::string& tname = classserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a LambdaLink, got %s", tname.c_str());
	}

}

DefineLink::DefineLink(const HandleSeq& oset,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(DEFINE, oset, tv, av)
{
	init(oset);
}

DefineLink::DefineLink(const Handle& name, const Handle& defn,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(DEFINE, HandleSeq({name, defn}), tv, av)
{
	init(getOutgoingSet());
}

DefineLink::DefineLink(Link &l)
	: Link(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, DEFINE))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a DefineLink, got %s", tname.c_str());
	}

	init(l.getOutgoingSet());
}

/* ===================== END OF FILE ===================== */
