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

namespace opencog {

class PatternMatch
{
	private:
		AtomSpace *atom_space;

		bool prt(Atom *);

		// Apply Filter rules, to create a normalized predicate.
		std::vector<Handle> normed_predicate;
		std::vector<Handle> bound_vars;

		bool apply_rule(Atom *);
		bool is_ling_rel(Atom *);

		// Examine each candidate for a match, in turn.
		bool do_candidate(Atom *);
		bool tree_compare(Atom *, Atom *);
		int depth;  // recursion depth for tree_compare.

		// Are two nodes instances of the same concept?
		bool concept_match(Atom *, Atom *);

		// Find the inheritance link in a set of incoming nodes
		Atom *get_general_concept(Atom *atom);
		Atom *concept_instance;
		Atom *general_concept;
		bool find_inheritance_link(Atom *);
		FollowLink fl;

		// Verify binding of varaibles
		int var_position(Atom *);

		// Embedding from subgraph to graph.
		std::map<Handle, Handle> morphism;

		// Result of solving the predicate
		std::vector<Handle> var_solution;
		std::vector<Handle> predicate_solution;

	public:
		PatternMatch(AtomSpace *);

		void filter(Handle, const std::vector<Handle> &);
		void match(void);

};
};

