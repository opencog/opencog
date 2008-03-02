/*
 * PatternMatch.h
 *
 * Linas Vepstas February 2008
 */

#ifndef OPENCOG_PATTERN_MATCH_H_
#define OPENCOG_PATTERN_MATCH_H_

#include <map>
#include <stack>

#include "types.h"
#include "Atom.h"
#include "Link.h"
#include "OutgoingTree.h"

namespace opencog {

class PatternMatch;

class PatternMatchCallback
{
	protected:
		std::map<Handle, Handle> *var_solution;
		std::map<Handle, Handle> *predicate_solution;
		friend class PatternMatch;

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
		virtual bool solution(void) = 0;
};

typedef std::vector<Handle> RootList;
typedef std::map<Handle, RootList *> RootMap;
typedef std::pair<Handle, RootList *> RootPair;

class PatternMatch
{
	private:
		AtomSpace *atom_space;

		static bool prt(Atom *);
		static bool prt(Handle);

		// -------------------------------------------
		// Setup the predicate to be solved.
		// Apply Filter rules, to create a normalized predicate.
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

		std::stack<Handle> pred_handle_stack;
		std::stack<Handle> soln_handle_stack;
		std::stack<Handle> root_handle_stack;
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> pred_solutn_stack;

		// -------------------------------------------

		PatternMatchCallback *pmc;
		// Result of solving the predicate
		std::map<Handle, Handle> var_solution;
		std::map<Handle, Handle> predicate_solution;

	public:
		PatternMatch(AtomSpace *);

		void match(PatternMatchCallback *,
		           std::vector<Handle> *preds,
		           std::vector<Handle> *vars);

		static void print_solution(std::map<Handle, Handle> &preds,
		                           std::map<Handle, Handle> &vars);

};
};

#endif /* OPENCOG_PATTERN_MATCH_H_ */
