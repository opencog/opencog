/*
 * PatternMatch.h
 *
 * Linas Vepstas February 2008
 */

#include <map>

#include "types.h"
#include "Atom.h"
#include "FollowLink.h"
#include "Link.h"
#include "OutgoingTree.h"

namespace opencog {

typedef std::vector<Handle> RootList;
typedef std::map<Handle, RootList *> RootMap;
typedef std::pair<Handle, RootList *> RootPair;

class PatternMatch
{
	private:
		AtomSpace *atom_space;

		bool prt(Atom *);

		// -------------------------------------------
		// Setup the predicate to be solved.
		// Apply Filter rules, to create a normalized predicate.
		std::vector<Handle> normed_predicate;
		std::map<Handle, bool> bound_vars;

		bool apply_rule(Atom *);
		bool is_ling_rel(Atom *);

		// -------------------------------------------
		// Traversal utilities
		RootMap root_map;
		Handle curr_root;
		bool note_root(Handle);

		// -------------------------------------------
		// Examine each candidate for a match, in turn.
		bool do_candidate(Atom *);

		// Recurisve tree comparison algorithm.
		bool tree_compare(Atom *, Atom *);
		int depth;  // recursion depth for tree_compare.

		// Tree comparison failed, erase the propsed solution.
		bool erase_solution(Handle);

		bool pred_up(Atom *);
		OutgoingTree ot;

		// -------------------------------------------
		// Routines that implement node matching heuristics.

		// Are two nodes instances of the same concept?
		bool concept_match(Atom *, Atom *);

		FollowLink fl;

		// Verify binding of variables
		bool is_var(Atom *);

		// -------------------------------------------

		// Result of solving the predicate
		std::map<Handle, Handle> var_solution;
		std::map<Handle, Handle> predicate_solution;

	public:
		PatternMatch(AtomSpace *);

		void filter(Handle, const std::vector<Handle> &);
		void match(void);

};
};

