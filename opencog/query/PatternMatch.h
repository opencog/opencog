/*
 * PatternMatch.h
 *
 * Linas Vepstas February 2008
 */

#ifndef _OPENCOG_PATTERN_MATCH_H
#define _OPENCOG_PATTERN_MATCH_H

#include <map>
#include <stack>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/query/OutgoingTree.h>

namespace opencog {

/**
 * Callback class, used to implement specifics of node 
 * matching, and also, to report solutions when found.
 */
class PatternMatchCallback
{
	public:
		virtual ~PatternMatchCallback() {};

		/**
		 * Called when two candidate nodes need to be
		 * compared. Return true if the nodes match,
		 * else return false.
		 */
		virtual bool node_match(Atom *, Atom *) = 0;

		/**
		 * Called when a solution is found. Should 
		 * return false to search for more solutions;
		 * or return true to terminate search.
		 */
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln) = 0;
};

typedef std::vector<Handle> RootList;
typedef std::map<Handle, RootList *> RootMap;
typedef std::pair<Handle, RootList *> RootPair;

class PatternMatch
{
	private:
		AtomSpace *atom_space;

		bool prt(Atom *);

		static void prtmsg(const char *, Atom *);
		static void prtmsg(const char *, Handle);

		// -------------------------------------------
		// predicates to be solved.
		std::vector<Handle> normed_predicate;
		std::set<Handle> bound_vars;

		// -------------------------------------------
		// Traversal utilities
		RootMap root_map;
		Handle curr_root;
		bool note_root(Handle);
		
		// -------------------------------------------
		// Examine each candidate for a match, in turn.
		bool do_candidate(Handle);

		// Recurisve tree comparison algorithm.
		bool tree_compare(Atom *, Atom *);
		int depth;  // recursion depth for tree_compare.

		bool pred_up(Handle);
		bool soln_up(Handle);
		OutgoingTree ot;
		Handle curr_soln_handle;
		Handle curr_pred_handle;
		void get_next_unsolved_pred(void);

		// Stack used during recursive exploration
		std::stack<Handle> pred_handle_stack;
		std::stack<Handle> soln_handle_stack;
		std::stack<Handle> root_handle_stack;
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> pred_solutn_stack;

		// -------------------------------------------

		// Result of solving the predicate
		std::map<Handle, Handle> var_solution;
		std::map<Handle, Handle> predicate_solution;

		// callback to report results.
		PatternMatchCallback *pmc;

	public:
		PatternMatch(void);
		void set_atomspace(AtomSpace *);

		void match(PatternMatchCallback *,
		           std::vector<Handle> *preds,
		           std::vector<Handle> *vars);

		static void print_solution(std::map<Handle, Handle> &preds,
		                           std::map<Handle, Handle> &vars);

};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_H
