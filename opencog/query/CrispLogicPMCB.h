/*
 * CrispLogicPMCB.h
 *
 * Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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
 *
 * Created by Linas Vepstas January, March 2009
 */

#ifndef _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
#define _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H

#include <opencog/atomspace/types.h>
#include <opencog/execution/EvaluationLink.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/DefaultPatternMatchCB.h>

namespace opencog {

/**
 * Callback mixin class, used to examine and work with SimpleTruthValues
 * so as to implement crisp, non-probabilistic Boolean logic
 * combinations of truth values.  That is, this code attempts to
 * combine the truth values in the predicate part of the pattern
 * match, using crisp-logic rules, in order to evaluate the possible
 * groundings of implications.
 *
 * This callback is meant to be chained into place: etc.
 */
class CrispLogicPMCB :
	public virtual DefaultPatternMatchCB
{
	public:
		CrispLogicPMCB(AtomSpace* as)
			: DefaultPatternMatchCB(as), _do_gpn(false) {}

		virtual bool node_match(Handle& npat_h, Handle& nsoln_h)
		{
			// If equality, then a match.
			if (npat_h != nsoln_h) return false;

			if (not _do_gpn)
				_do_gpn = (npat_h->getType() == GROUNDED_PREDICATE_NODE);
			return true;
		}

		virtual bool post_link_match(LinkPtr& pat_link, LinkPtr& gnd_link)
		{
			if (not _do_gpn) return true;
			_do_gpn = false;  // reset right away.

			// We will findoutselves here whenever the link contains
			// a GroundedPredicateNode. In such a case, execute the
			// node, and declare a match, or no match, depending
			// one how the evaluation turned out.  Its "crisp logic"
			// because we use a greater-than-half for the TV.
			TruthValuePtr tv(EvaluationLink::do_evaluate(_as, gnd_link->getHandle()));
			return tv->getMean() >= 0.5;
		}

		/**
 		 * This callback is called whenever a match has been identified
 		 * for a required assertion. Here, we check the truth value; if
 		 * it is >= 0.5, its taken to be true, and accepted, else, its 
 		 * taken to be false. 
 		 *
 		 * Return "true" to accept a clause, and return "false" to
 		 * reject it.
 		 */
		virtual bool clause_match(Handle& pattrn, Handle& grnd)
		{
			TruthValuePtr tv(grnd->getTruthValue());
			// printf (">>>>>>>> clause match tv=%f\n", tv.getMean());
			return tv->getMean() >= 0.5;
		}

		/**
 		 * This callback is called whenever a match has beeen identified
 		 * for a negated assertion (aka "optional clause"). Its "optional"
 		 * in the sense that, if its absent, its taken to be false, and
 		 * the pattern match is taken to be full-filled (because the pattern
 		 * was not found).  If the clause is found, then it is accepted only
 		 * if its truth value is *less* than 0.5.
 		 *
 		 * As usual, return "true" to accept a clause, and return "false" to
 		 * reject it. 
 		 */
		virtual bool optional_clause_match(Handle& pattrn, Handle& grnd)
		{
			// printf (">>>>>>>>>> hello optional term!! %p\n", grnd);
			if (Handle::UNDEFINED == grnd) return true;
			TruthValuePtr tv(grnd->getTruthValue());
			// printf (">>>> optional tv=%f\n", tv.getMean());
			return tv->getMean() < 0.5;
		}
	private:
		bool _do_gpn;
};

} // namespace opencog

#endif // _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
