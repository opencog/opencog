/*
 * SatisfactionLink.cc
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
#include <opencog/atomutils/FindUtils.h>

#include "PatternUtils.h"
#include "SatisfactionLink.h"

using namespace opencog;

void SatisfactionLink::init(void)
{
	// The LambdaLink constructor sets up _body and _varset
	unbundle_clauses(_body);
	setup_sat_body(_varset, _clauses);
}

/// The second half of the common initialization sequence
void SatisfactionLink::setup_sat_body(std::set<Handle>& vars,
                                      HandleSeq& clauses)
{
	validate_clauses(vars, clauses);
	std::set<Handle> evls;
	unbundle_virtual(vars, clauses,
                    _fixed, evls, _virtual);

	// Split the non virtual clauses into connected components
	get_connected_components(vars, _fixed, _components, _component_vars);
}

SatisfactionLink::SatisfactionLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: ConcreteLink(SATISFACTION_LINK, hseq, tv, av)
{
	init();
}

SatisfactionLink::SatisfactionLink(const Handle& vars, const Handle& body,
                   TruthValuePtr tv, AttentionValuePtr av)
	: ConcreteLink(SATISFACTION_LINK, HandleSeq({vars, body}), tv, av)
{
	init();
}

SatisfactionLink::SatisfactionLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: ConcreteLink(t, hseq, tv, av)
{
	// BindLink has a different clause initialization sequence
	if (SATISFACTION_LINK != t) return;
	init();
}

SatisfactionLink::SatisfactionLink(Link &l)
	: ConcreteLink(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, SATISFACTION_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a SatsifactionLink, got %s", tname.c_str());
	}

	// BindLink has a different clause sequence
	if (SATISFACTION_LINK != tscope) return;

	init();
}

/* ===================== END OF FILE ===================== */
