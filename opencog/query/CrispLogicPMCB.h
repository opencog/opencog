/*
 * CrispLogicPMCB.h
 *
 * Linas Vepstas January, March 2009
 */

#ifndef _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
#define _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
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

		/**
 		 * This callback is called whenever a match has beeen identified
 		 * for a required assertion. Here, we check the truth value; if
 		 * it is >0.5, its taken to be true, and accepted, else, its 
 		 * taken to be false. 
 		 *
 		 * Note the inverted return value: return "false" to accept 
 		 * a clause, and return "true" to reject it. 
 		 */
		virtual bool clause_match(Link *pattrn, Link *grnd)
		{
			const TruthValue &tv = grnd->getTruthValue();
			// printf (">>>>>>>> clause match tv=%f\n", tv.getMean());
			if (tv.getMean() < 0.5) return true;
			return false;
		}

		/**
 		 * This callback is called whenever a match has beeen identified
 		 * for a negated assertion (aka "optional clause"). Its "optional"
 		 * in the sense that, if its absent, its taken to be false, and
 		 * the pattern match is taken to be full-filled (because the pattern
 		 * was not found).  If the clause is found, then it is accepted only
 		 * if its truth value is *less* than 0.5. (i.e. it must be false to 
 		 * be accepted .. ergo, its a negation).
 		 *
 		 * As usual, return "false" to accept a clause, and return "true" to
 		 * reject it. 
 		 */
		virtual bool optional_clause_match(Link *pattrn, Link *grnd)
		{
			// printf (">>>>>>>>>> hello optional term!! %p\n", grnd);
			if (NULL == grnd) return false;
			const TruthValue &tv = grnd->getTruthValue();
			// printf (">>>> optional tv=%f\n", tv.getMean());
			if (tv.getMean() > 0.5) return true;
			return false;
		}
};

} // namespace opencog

#endif // _OPENCOG_CRISP_LOGIC_PATTERN_MATCH_H
