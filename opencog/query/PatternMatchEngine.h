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
	typedef std::map<Handle, RootList> ConnectMap;
	typedef std::pair<Handle, RootList> ConnectPair;

	private:
		// -------------------------------------------
		// The current set of clauses (redex context) being grounded.
		void setup_redex(const std::set<Handle> &vars,
		                   const std::vector<Handle> &component);
		void clear_redex(const std::string& name = "topmost level");
		bool explore_redex(const Handle&, const Handle&, const Handle&);

		std::string _redex_name;  // for debugging only!

		// variables that need to be grounded.
		std::set<Handle> _bound_vars;

		// List of clauses that need to be grounded.
		HandleSeq        _cnf_clauses;
		HandleSeq        _mandatory;
		std::set<Handle> _optionals;
		std::set<Handle> _evaluatable;
		ConnectMap       _connectivity_map;   // initialized by make_root_map()

		void make_connectivity_map(const Handle&, const Handle&);

		// -------------------------------------------
		// Recursive redex support. These are stacks of the clauses
		// above, that are being searched.
		std::stack<std::string>      _stack_redex_name;  // for debugging only
		std::stack<std::set<Handle>> _stack_bound_vars;
		std::stack<HandleSeq>        _stack_cnf_clauses;
		std::stack<HandleSeq>        _stack_mandatory;
		std::stack<std::set<Handle>> _stack_optionals;
		std::stack<std::set<Handle>> _stack_evaluatable;
		std::stack<ConnectMap>       _stack_connectivity_map;

		void push_redex(void);
		void pop_redex(void);

		// --------------------------------------------
		// Current traversal state

		// Map of current groundings of variables to thier grounds
		// Also contains grounds of subclauses (not sure why, this seems
		// to be needed)
		std::map<Handle, Handle> var_grounding;
		// Map of clauses to their current groundings
		std::map<Handle, Handle> clause_grounding;

		// Set of clauses for which a grounding is currently being attempted.
		typedef std::set<Handle> IssuedSet;
		IssuedSet issued;     // stacked on issued_stack

		int depth;      // Recursion depth for tree_compare.
		bool in_quote;  // Everything is literal in a quote.

		Handle curr_root;         // stacked onto root_handle_stack
		Handle curr_soln_handle;  // stacked onto soln_handle_stack
		Handle curr_pred_handle;  // stacked onto pred_handle_stack

		void clear_current_state(void);  // clear the stuff above

		// -------------------------------------------
		// Stack used to store current traversal state during
		// backtracking, to find additional groundings
		std::stack<Handle> root_handle_stack;
		std::stack<Handle> pred_handle_stack;
		std::stack<Handle> soln_handle_stack;

		// Stacks containing partial groundings.
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> var_solutn_stack;
		std::stack<SolnMap> pred_solutn_stack;

		std::stack<IssuedSet> issued_stack;
		std::stack<bool> in_quote_stack;

		// push, pop and clear these states.
		void graph_stacks_push(void);
		void graph_stacks_pop(void);
		void graph_stacks_clear(void);
		unsigned int _graph_stack_depth;

		// -------------------------------------------
		// Recursive tree comparison algorithm.
		bool tree_compare(const Handle&, const Handle&);
		bool redex_compare(const LinkPtr&, const LinkPtr&);

		bool pred_up(const Handle&);
		bool soln_up(const Handle&);
		bool do_soln_up(const Handle&); // See PatternMatchEngine.cc for comment
		bool clause_accepted;
		void get_next_untried_clause(void);
		bool get_next_untried_helper(bool);

		// --------------------------------------------------
		// Unordered-link stuff. This needs a major overhaul.
		// Stacks are used to explore all possible permuations of
		// unordered links, but this is incorectly/incompletely
		// designed.
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
		// Callback to whom the results are reported.
		PatternMatchCallback *_pmc;

	public:
		PatternMatchEngine(void) {}

		// Examine the locally connected neighborhood for possible
		// matches.
		bool explore_neighborhood(const Handle&, const Handle&, const Handle&);

		// Do the actual pattern search.
		void match(PatternMatchCallback *,
		           const std::set<Handle> &vars,
		           const std::vector<Handle> &component);

		// Handy-dandy utilities
		static void print_solution(const std::map<Handle, Handle> &vars,
		                           const std::map<Handle, Handle> &clauses);

		static void print_predicate(const std::set<Handle> &vars,
		                            const std::vector<Handle> &clauses);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_ENGINE_H
