/*
 * CrispLogicPMCB.h
 *
 * Copyright (C) 2009, 2015 Linas Vepstas <linasvepstas@gmail.com>
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
#include <opencog/atoms/execution/EvaluationLink.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/DefaultPatternMatchCB.h>

namespace opencog {

/**
 * Callback mix-in class, used to examine and work with SimpleTruthValues
 * so as to implement crisp, non-probabilistic Boolean logic
 * combinations of truth values.  That is, this code attempts to
 * combine the truth values in the clause part of the pattern
 * match, using crisp-logic rules, in order to evaluate the possible
 * groundings of implications.
 *
 * Right now, this is sort of quirky in how it behaves... and perhaps
 * should be changed. For now, it works for the foreseeable
 * applications. Sooo:
 *
 * We don't examine the truth value of every node and link, to determine
 * a match. Perhaps we should....  We do examine the truth values of the
 * clauses, to judge whether a clause as a wole, is a matfch or not.
 * Also, we examine the truth value of links, but only if they are in
 * a SequentialAnd, AND ALSO if they are evaluatable.   Evaluatable
 * links can be used as stop-go markers in a pattern, so that's good.
 * However, the nature of the pattern search algo means that some of
 * these might be visited twice.  Thus, the SequentialAnd is used to
 * guarantee that each is visited only once (or not at all), thus giving
 * the impression of a deterministic sequence of execution. Without
 * the SequentialAnd, some link in the middle might be searched first,
 * as that is where the neighborhood search just happened to start.
 * Only later is the SequentialAnd discovered, and, after it has, it
 * will explored in order.
 */
class CrispLogicPMCB :
	public virtual DefaultPatternMatchCB
{
	public:
		CrispLogicPMCB(AtomSpace* as)
			: DefaultPatternMatchCB(as) {}

		/**
 		 * This callback is called whenever a match has been identified
		 * for a required assertion. Here, we check the truth value; if
		 * it is >= 0.5, its taken to be true, and accepted, else, its
		 * taken to be false.
		 *
		 * Return "true" to accept a clause, and return "false" to
		 * reject it.
		 */
		virtual bool clause_match(const Handle& pattrn, const Handle& grnd)
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
		virtual bool optional_clause_match(const Handle& pattrn, const Handle& grnd)
		{
			// printf (">>>>>>>>>> hello optional term!! %p\n", grnd);
			if (Handle::UNDEFINED == grnd) return true;
			TruthValuePtr tv(grnd->getTruthValue());
			// printf (">>>> optional tv=%f\n", tv.getMean());
			return tv->getMean() < 0.5;
		}
};

} // namespace opencog

#endif // _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
