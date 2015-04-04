/*
 * PatternMatchEngine.h
 *
 * Author: Linas Vepstas February 2008
 *
 * Copyright (C) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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
 */

#ifndef _OPENCOG_PATTERN_MATCH_ENGINE_H
#define _OPENCOG_PATTERN_MATCH_ENGINE_H

#include <map>
#include <set>
#include <stack>
#include <vector>

#include <opencog/query/PatternMatchCallback.h>

namespace opencog {

class PatternMatchEngine
{
	// Private, locally scoped typedefs, not used outside of this class.
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList> RootMap;
	typedef std::pair<Handle, RootList> RootPair;

	private:
		// -------------------------------------------
		// predicate to be solved.
		std::set<Handle> _bound_vars;
		std::vector<Handle> _cnf_clauses;
		std::set<Handle> _optionals;

		std::set<Handle> _evaluatable;

		// -------------------------------------------
		// Traversal utilities
		RootMap _root_map;
		Handle curr_root;
		bool note_root(const Handle&);
		
		// -------------------------------------------
		// Recursive tree comparison algorithm.
		bool tree_compare(const Handle&, const Handle&);
		bool compose_compare(const LinkPtr&, const LinkPtr&);
		int depth;      // Recursion depth for tree_compare.
		bool in_quote;  // Everything is literal in a quote.

		bool pred_up(const Handle&);
		bool soln_up(const Handle&);
		bool do_soln_up(const Handle&); // See PatternMatchEngine.cc for comment
		bool clause_accepted;
		Handle curr_soln_handle;
		Handle curr_pred_handle;
		void get_next_untried_clause(void);
		bool get_next_untried_helper(bool);

		// Stack used during recursive exploration.
		std::stack<Handle> pred_handle_stack;
		std::stack<Handle> soln_handle_stack;
		std::stack<Handle> root_handle_stack;
		std::stack<bool> in_quote_stack;
		unsigned int stack_depth;

		// Stacks containing partial groundings.
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> pred_solutn_stack;
		std::stack<SolnMap> var_solutn_stack;

		// Set of clauses for which a grounding is currently being attempted.
		typedef std::set<Handle> IssuedSet;
		IssuedSet issued;
		std::stack<IssuedSet> issued_stack;

		// Stacks used to explore all possible permuations of
		// unordered links. 
		bool have_more;
		size_t more_depth;

		// Substacks used for nested unorderered links.
		typedef std::vector<bool> MoreStack;
		MoreStack more_stack;
		typedef std::vector<Handle> Permutation;
		typedef std::stack<Permutation> PermuStack;
		PermuStack mute_stack;

		// Stacks used for unordered links in different clauses.
		std::stack<bool> have_stack;
		std::stack<size_t> depth_stack;
		std::stack<MoreStack> unordered_stack;
		std::stack<PermuStack> permutation_stack;

		// -------------------------------------------

		// Result of solving the predicate
		// Map variables (and sub-clauses as well) to their groundings
		std::map<Handle, Handle> var_grounding;
		// Map clauses to their groundings
		std::map<Handle, Handle> clause_grounding;

		void clear_state(void);

		// Callback to whom the results are reported.
		PatternMatchCallback *pmc;

	public:
		PatternMatchEngine(void) {}

		// Clear all internal state.
		void clear(void);

		// Examine each candidate for a match, in turn.
		bool do_candidate(const Handle&, const Handle&, const Handle&);

		// Do the actual pattern search.
		void match(PatternMatchCallback *,
		           const std::set<Handle> &vars,
		           const std::vector<Handle> &clauses,
		           const std::vector<Handle> &negations);

		// Handy-dandy utilities
		static void print_solution(const std::map<Handle, Handle> &vars,
		                           const std::map<Handle, Handle> &clauses);

		static void print_predicate(const std::set<Handle> &vars,
		                            const std::vector<Handle> &clauses);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_ENGINE_H
