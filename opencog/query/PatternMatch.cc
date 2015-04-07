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
   if (check_conn and 0 == _virtual.size()) check_connectivity(_components);
   PatternMatch::do_match(pmc, _varset, _virtual,
	                       _components, _component_vars);
}

// All clauses of the Concrete link are connected, so this is easy.
void ConcreteLink::satisfy(PatternMatchCallback* pmcb)
{
   PatternMatchEngine pme;
	pme.match(pmcb, _varset, ... 
}

void SatisfactionLink::satisfy(PatternMatchCallback* pmc)
{
   PatternMatch::do_match(pmc, _varset, _virtual,
	                       _components, _component_vars);
}

void BetaRedex::satisfy(PatternMatchCallback* pmc,
                          const HandleSeq& args)
{
// this is junk, I think..... think about it a bit, then remove me....
printf ("duuuuuuuuuuuuude called the compose satter\n");
}

/* ===================== END OF FILE ===================== */
