/*
 * ComposeLink.cc
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
#include <opencog/atoms/bind/DefineLink.h>

#include "ComposeLink.h"

using namespace opencog;

void ComposeLink::init(const HandleSeq& oset)
{
	// Must have name and body
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"Expecting name and a list of values, got size %d", oset.size());

	Type ot = oset[1]->getType();
	if (not classserver().isA(ot, ORDERED_LINK))
      throw InvalidParamException(TRACE_INFO,
         "Expecting and ordered list of arguments!");
}

ComposeLink::ComposeLink(const HandleSeq& oset,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(COMPOSE_LINK, oset, tv, av)
{
	init(oset);
}

ComposeLink::ComposeLink(const Handle& name, const Handle& defn,
                       TruthValuePtr tv, AttentionValuePtr av)
	: Link(COMPOSE_LINK, HandleSeq({name, defn}), tv, av)
{
	init(getOutgoingSet());
}

ComposeLink::ComposeLink(Link &l)
	: Link(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, COMPOSE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a ComposeLink, got %s", tname.c_str());
	}

	init(l.getOutgoingSet());
}

/// Simply return the arguments to be composed.
const HandleSeq& ComposeLink::getArgs(void) const
{
	LinkPtr args(LinkCast(_outgoing[1]));
	return args->getOutgoingSet();
}

/// Compose this link with the defined link, and return the result.
Handle ComposeLink::compose(void)
{
	// Is the defined link actually defined?
	// We cannot do this check when the ctor runs, since it might
	// not yet be defined (or the definition may have changed!!)
   HandleSeq ename;
   _outgoing[0]->getIncomingSetByType(std::back_inserter(ename), DEFINE_LINK);
   if (1 != ename.size())
      throw InvalidParamException(TRACE_INFO,
         "Cannot compose with an undefined function!");

	// Get the the defined function
	DefineLinkPtr ldefun(DefineLinkCast(ename[0]));
	LambdaLinkPtr lam(ldefun->get_definition());

	// Substitute the arguments
	return lam->substitute(getArgs());
}

/* ===================== END OF FILE ===================== */
