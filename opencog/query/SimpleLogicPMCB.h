/*
 * SimpleLogicPMCB.h
 *
 * Linas Vepstas January, March 2009
 */

#ifndef _OPENCOG_SIMPLE_LOGIC_PATTERN_MATCH_H
#define _OPENCOG_SIMPLE_LOGIC_PATTERN_MATCH_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog {

/**
 * Callback mixin class, used to examine and work with SimpleTruthValues
 * so as to implement basic term logic-like (boolean logic-like)
 * combinations of truth values.  That is, this code attempts to
 * combine the truth values in the predicate part of the pattern
 * match, in order to arrive at reasonable TruthValues for any
 * given grounding.
 *
 * This callback is meant to be chained into place: etc.
 */
class SimpleLogicPMCB :
	public virtual PatternMatchCallback
{
	public:
		/**
		 * Called when a node in the template pattern
		 * needs to be compared to a possibly matching
		 * node in the atomspace. The first argument
		 * is a node from the pattern, and the second
		 * is a possible solution node from the atomspace.
		 * Return false if the nodes match, else return
		 * true. (i.e. return true if mis-match).
		 *
		 * By default, the nodes must be identical,
		 * or one of them must be a variable.
		 */
		virtual bool node_match(Node *npat, Node *ngrnd)
		{
			// If equality, then a match.
			if (npat == ngrnd) return false;

			// If the ungrounded term is a variable, then OK.
			Type pattype = npat->getType();
			if (pattype == VARIABLE_NODE) return false;

			return true;
		}


		/**
		 * Called when a link in the template pattern
		 * needs to be compared to a possibly matching
		 * link in the atomspace. The first argument
		 * is a link from the pattern, and the second
		 * is a possible solution link from the atomspace.
		 * Return false if the links match, else return
		 * true. (i.e. return true if mis-match).
		 *
		 * By default, the link arity and the 
		 * link types must match.
		 */
		virtual bool link_match(Link *lpat, Link *lgnd)
		{
			if (lpat == lgnd) return false;

			Type pattype = lpat->getType();
			Type gndtype = lgnd->getType();
printf ("comp link type %d(%s) %d(%s)\n", pattype,  ClassServer::getTypeName(pattype).c_str(),
gndtype, ClassServer::getTypeName(gndtype).c_str());
			if (NOT_LINK == pattype)
			{
printf("ola have not link\n");
			}
			if (lpat->getArity() != lgnd->getArity()) return true;

			// If types differ, no match,
			if ((pattype != VARIABLE_SCOPE_LINK) && 
			    (pattype != gndtype)) return true;
			return false;
		}

		virtual bool tree_match(Link *pattrn, Link *grnd)
		{
			const TruthValue &tv = grnd->getTruthValue();
printf ("hello world str=%f\n", tv.getMean());
			return false;
		}
};

} // namespace opencog

#endif // _OPENCOG_SIMPLE_LOGIC_PATTERN_MATCH_H
