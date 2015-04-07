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
	LambdaLink::init(_outgoing);
	unbundle_clauses(_body);
	setup_sat_body();
}

/// The second half of the common initialization sequence
void SatisfactionLink::setup_sat_body(void)
{
	validate_clauses(_varset, _clauses);
	extract_optionals(_varset, _clauses);
	unbundle_virtual(_varset, _cnf_clauses,
                    _fixed, _virtual);

	// Split the non virtual clauses into connected components
	std::vector<HandleSeq> comps;
	std::vector<std::set<Handle>> comp_vars;
	get_connected_components(_varset, _fixed, comps, comp_vars);

	// If there are no virtuals, and there is only one connected
	// component, then this is just a single Concrete link; perform
	// the rest of initialization for just that.  There is a pathological
	// case where there are no virtuals, but there are multiple
	// disconnected components.  I think that this is a user-error,
	// but in fact PLN does have a rule which wants to explore that
	// combinatoric explosion, on purpose. So we have to allow the
	// multiple disconnected components for that case.
	_num_comps = comps.size();
	_num_virts = _virtual.size();
	if (0 == _num_virts and 1 == _num_comps)
	{
		make_connectivity_map(_cnf_clauses);
		return;
	}

	// If we are here, then set up a ConcreteLink for each connected
	// component.  Use emplace_back to avoid a copy.
	_components.reserve(_num_comps);
	for (size_t i=0; i<_num_comps; i++)
	{
		Handle h(createConcreteLink(comp_vars[i], _typemap,
		                            comps[i], _optionals));
		_components.push_back(h);
	}
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
