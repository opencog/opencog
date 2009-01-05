/*
 * DefaultPatternMatchCB.h
 *
 * Linas Vepstas January 2009
 */

#ifndef _OPENCOG_DEFAULT_PATTERN_MATCH_H
#define _OPENCOG_DEFAULT_PATTERN_MATCH_H

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/query/PatternMatch.h>

namespace opencog {

/**
 * Callback class, used to implement specifics of node 
 * matching, and also, to report solutions when found.
 */
class DefaultPatternMatchCB :
	public PatternMatchCallback
{
	public:
		virtual ~DefaultPatternMatchCB() {};

		/**
		 * Called when a node in the template pattern
		 * needs to be compared to a possibly matching
		 * node in the atomspace. The first argument
		 * is a node from the pattern, and the second
		 * is a possible solution node from the atomspace.
		 * Return false if the nodes match, else return
		 * true. (i.e. return true if mis-match).
		 */
		virtual bool node_match(Atom *apat, Atom *asoln)
		{
			if (apat == asoln) return false;
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
		 */
		virtual bool link_match(Atom *lpat, Atom *lsoln)
		{
			return false;
		}

		/**
		 * Called when a solution is found. Should 
		 * return false to search for more solutions;
		 * or return true to terminate search.
		 */
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln) = 0;
};

} // namespace opencog

#endif // _OPENCOG_DEFAULT_PATTERN_MATCH_H
