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
			: DefaultPatternMatchCB(as), _in_seq_and(false) {}

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

		virtual bool link_match(const LinkPtr& lpat, const LinkPtr& lsoln)
		{
			Type pattype = lpat->getType();
			if (SEQUENTIAL_AND_LINK == pattype) _in_seq_and = true;

			// If the pattern is exactly the same link as the proposed
			// grounding, then its a perfect match.
			if (lpat == lsoln) return true;

			if (lpat->getArity() != lsoln->getArity()) return false;
			Type soltype = lsoln->getType();

			// If types differ, no match
			return pattype == soltype;
		}

		virtual bool post_link_match(const LinkPtr& pat_link,
		                             const LinkPtr& gnd_link)
		{
			if (not _in_seq_and) return true;

			Type pattype = pat_link->getType();
			if (SEQUENTIAL_AND_LINK == pattype)
			{
				_in_seq_and = false;
				return true;
			}

			Handle hp(pat_link);
			if (_dynamic->find(hp) == _dynamic->end()) return true;

			// We will find ourselves here whenever the link contains
			// a GroundedPredicateNode. In this case, execute the
			// node, and declare a match, or no match, depending
			// one how the evaluation turned out.  Its "crisp logic"
			// because we use a greater-than-half for the TV.
			//
			// This is similar to evaluate_link(), except that,
			// in the current design, this one will get called instead
			// of that one, when, the pattern has no variables in it.
			// Perhaps this is wrong .. XXX FIXME
			TruthValuePtr tv(EvaluationLink::do_evaluate(_as, gnd_link->getHandle()));
			return tv->getMean() >= 0.5;
		}

		virtual bool initiate_search(PatternMatchEngine* pme,
		                             const std::set<Handle> &vars,
		                             const std::vector<Handle> &clauses)
		{
			// Extract the GPN's. We will need these during the search.
			_in_seq_and = false;
			return DefaultPatternMatchCB::initiate_search(pme, vars, clauses);
		}

	private:
		bool _in_seq_and;
		// XXX FIXME the boolean _in_seq_and should really be a stack,
		// which is pushed/popped on each entry and exit.  The stack is
		// needed for recursive nesting of these things.  Right now,
		// I'm being lazy, so that's a bug ....
};

} // namespace opencog

#endif // _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
