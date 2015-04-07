/*
 * PatternMatch.cc
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
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

#include <opencog/atoms/bind/BindLink.h>
#include <opencog/atoms/bind/BetaRedex.h>
#include <opencog/atoms/bind/ConcreteLink.h>
#include <opencog/atoms/bind/SatisfactionLink.h>
#include <opencog/util/Logger.h>

#include "PatternMatch.h"
#include "PatternMatchEngine.h"

using namespace opencog;

#ifdef DEBUG
   #define dbgprt(f, varargs...) printf(f, ##varargs)
#else
   #define dbgprt(f, varargs...)
#endif

/* ================================================================= */
/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Given a BindLink containing variable declarations and an
 * ImplicationLink, this method will "evaluate" the implication,
 * matching the predicate, and creating a grounded implicand,
 * assuming the predicate can be satisfied. Thus, for example,
 * given the structure
 *
 *    BindLink
 *       ListLink
 *          VariableNode "$var0"
 *          VariableNode "$var1"
 *       ImplicationLink
 *          AndList
 *             etc ...
 *
 * Evaluation proceeds as decribed in the "do_imply()" function below.
 * The whole point of the BindLink is to do nothing more than
 * to indicate the bindings of the variables, and (optionally) limit
 * the types of acceptable groundings for the variables.
 */
void BindLink::imply(PatternMatchCallback* pmc, bool check_conn)
{
   if (check_conn and 0 < _components.size())
		throw InvalidParamException(TRACE_INFO,
			"BindLink consists of multiple disconnected components!");
			// ConcreteLink::check_connectivity(_comps);

	SatisfactionLink::satisfy(pmc);
}

// All clauses of the Concrete link are connected, so this is easy.
void ConcreteLink::satisfy(PatternMatchCallback* pmcb,
                           PatternMatchEngine *pme) const
{
	pme->_bound_vars = _varset;
	pme->_cnf_clauses = _cnf_clauses;
	pme->_mandatory = _mandatory;
	pme->_optionals = _optionals;
	pme->_evaluatable = _evaluatable;
	pme->_connectivity_map = _connectivity_map;
	pme->_pmc = pmcb;

	pmcb->set_type_restrictions(_typemap);
	pmcb->initiate_search(pme, _varset, _mandatory);
}

void ConcreteLink::satisfy(PatternMatchCallback* pmcb) const
{
   PatternMatchEngine pme;
	satisfy(pmcb, &pme);
}

void SatisfactionLink::satisfy(PatternMatchCallback* pmcb) const
{
	if (1 == _num_comps)
	{
		ConcreteLink::satisfy(pmcb);
		return;
	}

	// If we are here, then we've got a knot in the center of it all.
	// Removing the virtual clauses from the hypergraph typically causes
	// the hypergraph to fall apart into multiple components, (i.e. none
	// are connected to one another). The virtual clauses tie all of
	// these back together into a single connected graph.
	//
	// There are several solution strategies possible at this point.
	// The one that we will pursue, for now, is to first ground all of
	// the distinct components individually, and then run each possible
	// grounding combination through the virtual link, for the final
	// accept/reject determination.

	std::vector<std::vector<std::map<Handle, Handle>>> comp_pred_gnds;
	std::vector<std::vector<std::map<Handle, Handle>>> comp_var_gnds;

	for (size_t i=0; i<_num_comps; i++)
	{
		// Pass through the callbacks, collect up answers.
		PMCGroundings gcb(pmcb);
		PatternMatchEngine pme;
		ConcreteLinkPtr clp(ConcreteLinkCast(_components[i]));
		clp->satisfy(&gcb, &pme);

		comp_var_gnds.push_back(gcb._var_groundings);
		comp_pred_gnds.push_back(gcb._pred_groundings);
	}

	// And now, try grounding each of the virtual clauses.
	dbgprt("BEGIN component recursion: ====================== "
	       "num comp=%zd num virts=%zd\n",
	       comp_var_gnds.size(), virtuals.size());
	std::map<Handle, Handle> empty_vg;
	std::map<Handle, Handle> empty_pg;
	std::vector<Handle> negations; // currently ignored
	PatternMatch::recursive_virtual(pmcb, virtuals, negations,
	                  empty_vg, empty_pg,
	                  comp_var_gnds, comp_pred_gnds);
}

void BetaRedex::satisfy(PatternMatchCallback* pmc,
                          const HandleSeq& args)
{
// this is junk, I think..... think about it a bit, then remove me....
printf ("duuuuuuuuuuuuude called the compose satter\n");
}

/* ===================== END OF FILE ===================== */
