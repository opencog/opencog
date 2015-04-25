/*
 * BindLink.cc
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

#include "BindLink.h"

using namespace opencog;

void BindLink::init(void)
{
	extract_variables(_outgoing);
	validate_body(_body);
	unbundle_clauses(_hclauses);

	// Remainder of the init is just like in the SatisfactionLink
	setup_sat_body();
	_pat.redex_name = "anonymous BindLink";
}

BindLink::BindLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: SatisfactionLink(BIND_LINK, hseq, tv, av)
{
	init();
}

BindLink::BindLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: SatisfactionLink(t, hseq, tv, av)
{
	init();
}

BindLink::BindLink(Link &l)
	: SatisfactionLink(l)
{
	Type t = l.getType();
	if (not classserver().isA(t, BIND_LINK))
	{
		const std::string& tname = classserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a BindLink, got %s", tname.c_str());
	}

	init();
}

/* ================================================================= */
/**
 * Validate the body for syntax correctness.
 *
 * Given an ImplicatioLink, this will check to make sure that
 * it is of the appropriate structure: that it consists of two
 * parts: a set of clauses, and an implicand.  That is, it must
 * have the structure:
 *
 *    ImplicationLink
 *       SomeLink
 *       AnotherLink
 *
 * The conents of "SomeLink" is not validated here, it is
 * validated by validate_clauses()
 *
 * As a side-effect, if SomeLink is an AndLink, the list of clauses
 * is unpacked.
 */
void BindLink::validate_body(const Handle& hbody)
{
	// Type must be as expected
	if (IMPLICATION_LINK != hbody->getType())
		throw InvalidParamException(TRACE_INFO,
			"Bindlink expects an ImplicationLink, got %s",
			classserver().getTypeName(hbody->getType()).c_str());

	LinkPtr lbody(LinkCast(hbody));
	const std::vector<Handle>& oset = lbody->getOutgoingSet();
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"ImplicationLink has wrong size: %d", oset.size());

	_hclauses = oset[0];
	_implicand = oset[1];
}

/* ===================== END OF FILE ===================== */
