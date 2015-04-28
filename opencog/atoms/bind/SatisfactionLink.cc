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
	extract_variables(_outgoing);
	unbundle_clauses(_body);
	setup_sat_body();
}

/// The second half of the common initialization sequence
void SatisfactionLink::setup_sat_body(void)
{
	_pat.redex_name = "anonymous SatisfactionLink";
	validate_clauses(_varlist.varset, _pat.clauses);
	extract_optionals(_varlist.varset, _pat.clauses);
	unbundle_virtual(_varlist.varset, _pat.cnf_clauses,
                    _fixed, _virtual, _pat.black);

	// Split the non-virtual clauses into connected components
	std::vector<HandleSeq> comps;
	std::vector<std::set<Handle>> comp_vars;
	get_connected_components(_varlist.varset, _fixed, comps, comp_vars);

	// If there is only one connected component, then this can be handled
	// during search by a single Concrete link. The multi-clause grounding
	// mechanism is not required for that case.
	_num_comps = comps.size();
	_num_virts = _virtual.size();
	if (1 == _num_comps)
	{
		make_connectivity_map(_pat.cnf_clauses);
		return;
	}

	// If we are here, then set up a ConcreteLink for each connected
	// component.  Use emplace_back to avoid a copy.
	//
	// There is a pathological case where there are no virtuals, but
	// there are multiple disconnected components.  I think that this is
	// a user-error, but in fact PLN does have a rule which wants to
	// explore that combinatoric explosion, on purpose. So we have to
	// allow the multiple disconnected components for that case.
	_components.reserve(_num_comps);
	for (size_t i=0; i<_num_comps; i++)
	{
		Handle h(createConcreteLink(comp_vars[i], _varlist.typemap,
		                            comps[i], _pat.optionals));
		_components.push_back(h);
	}
}

SatisfactionLink::SatisfactionLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: ConcreteLink(SATISFACTION_LINK, hseq, tv, av)
{
	init();
}

SatisfactionLink::SatisfactionLink(const Handle& body,
                   TruthValuePtr tv, AttentionValuePtr av)
	: ConcreteLink(SATISFACTION_LINK, HandleSeq({body}), tv, av)
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

/// Constructor that takes a pre-determined set of variables, and
/// a list of clauses to solve.  This is currently kind-of crippled,
/// since no variable type restricions are possible, and no optionals,
/// either.  By contrast, the ConcreteLink constructor does allw these
/// things, but it does not allow virtual links. Alas.
SatisfactionLink::SatisfactionLink(const std::set<Handle>& vars,
                                   const HandleSeq& clauses)
	: ConcreteLink(SATISFACTION_LINK, HandleSeq())
{
	_varlist.varset = vars;
	_pat.clauses = clauses;
	setup_sat_body();
}

/* ===================== END OF FILE ===================== */
