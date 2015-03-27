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
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/query/DefaultPatternMatchCB.h>
#include <opencog/atomutils/PatternUtils.h>

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

		virtual bool link_match(LinkPtr& lpat, LinkPtr& lsoln)
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

		virtual bool post_link_match(LinkPtr& pat_link, LinkPtr& gnd_link)
		{
			if (not _in_seq_and) return true;

			Type pattype = pat_link->getType();
			if (SEQUENTIAL_AND_LINK == pattype)
			{
				_in_seq_and = false;
				return true;
			}

			Handle hp(pat_link);
			if (_dyns.find(hp) == _dyns.end()) return true;

			// We will find ourselves here whenever the link contains
			// a GroundedPredicateNode. In this case, execute the
			// node, and declare a match, or no match, depending
			// one how the evaluation turned out.  Its "crisp logic"
			// because we use a greater-than-half for the TV.
			//
			// This is similar to virtual_link_match(), except that,
			// in the current design, this one will get called instead
			// of that one, when, the pattern has no variables in it.
			// Perhaps this is wrong .. XXX FIXME
			TruthValuePtr tv(EvaluationLink::do_evaluate(_as, gnd_link->getHandle()));
			return tv->getMean() >= 0.5;
		}

		virtual void perform_search(PatternMatchEngine* pme,
		                            std::set<Handle> &vars,
		                            std::vector<Handle> &clauses,
		                            std::vector<Handle> &negations)
		{
			// Extract the GPN's. We will need these during the search.
			_in_seq_and = false;
			FindVariables fv(GROUNDED_PREDICATE_NODE, false);
			fv.find_vars(clauses);
			fv.find_vars(negations);
			_dyns = fv.holders;
			DefaultPatternMatchCB::perform_search(pme, vars, clauses, negations);
		}

	private:
		// Info about GPN's
		std::set<Handle> _dyns;
		bool _in_seq_and;
		// XXX FIXME the boolean _in_seq_and should really be a stack,
		// which is pushed/popped on each entry and exit.  The stack is
		// needed for recursive nesting of these things.  Right now,
		// Im being lazy, but that's a bug ....
};

} // namespace opencog

#endif // _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
