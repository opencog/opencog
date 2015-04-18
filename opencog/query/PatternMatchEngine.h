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
#include <unordered_map>
#include <vector>

#include <opencog/query/Pattern.h>
#include <opencog/query/PatternMatchCallback.h>
#include <opencog/atomspace/ClassServer.h>

namespace opencog {

class PatternMatchEngine
{
	// -------------------------------------------
	// Callback to whom the results are reported.
	PatternMatchCallback &_pmc;
	ClassServer& _classserver;

	// Private, locally scoped typedefs, not used outside of this class.
	// Used for managing ChoiceLink state
	typedef std::pair<const Handle&, const Handle&> Choice;
	typedef std::map<Choice, size_t> ChoiceState;

	private:
		// -------------------------------------------
		// The current set of clauses (redex context) being grounded.
		// A single redex consists of a collection of clauses, all of
		// which must be grounded.
		bool explore_redex(const Handle&, const Handle&, const Handle&);

		// These have to be pointers, not references; they get pushed
		// onto a stack when a new redex context is started. This is
		// how redex recursion will (eventually) be implemented.
		const Variables* _varlist;
		const Pattern* _pat;

		bool is_optional(const Handle& h) {
			return (_pat->optionals.count(h) != 0); }

		bool is_evaluatable(const Handle& h) {
			return (_pat->evaluatable_holders.count(h) != 0); }

		bool is_black(const Handle& h) {
			return (_pat->black.count(h) != 0); }

		// -------------------------------------------
		// Recursive redex support. These are stacks of the clauses
		// above, that are being searched.
		std::stack<const Variables*>  _stack_variables;
		std::stack<const Pattern*>    _stack_pattern;

		void push_redex(void);
		void pop_redex(void);

		// --------------------------------------------
		// Current clause traversal state. These hold the state needed
		// to traverse a single clause, and find groundings for it.
		// Note, though, that these are cumulative: so e.g. the
		// var_grounding map accumulates variable groundings for this
		// clause, and all previous clauses so far.

		// Map of current groundings of variables to thier grounds
		// Also contains grounds of subclauses (not sure why, this seems
		// to be needed)
		std::map<Handle, Handle> var_grounding;
		// Map of clauses to their current groundings
		std::map<Handle, Handle> clause_grounding;

		// Set of clauses for which a grounding is currently being attempted.
		typedef std::set<Handle> IssuedSet;
		IssuedSet issued;     // stacked on issued_stack

		unsigned int depth; // Recursion depth for tree_compare.
		bool in_quote;      // Everything is literal in a quote.

		Handle curr_root;         // stacked onto root_handle_stack
		Handle curr_soln_handle;  // stacked onto soln_handle_stack
		Handle curr_term_handle;  // stacked onto term_handle_stack

		// OrLink (choice) state management
		ChoiceState _choice_state;
		size_t next_choice(const Handle&, const Handle&);
		bool _need_choice_push;

		void clear_current_state(void);  // clear the stuff above

		// -------------------------------------------
		// Stack used to store current traversal state for a single
		// clause. These are pushed when a clause is fully grounded,
		// and a new clause is about to be started. These are popped
		// in order to get back to the original clause, and resume
		// traversal of that clause, where it was last left off.
		std::stack<Handle> root_handle_stack;
		std::stack<Handle> term_handle_stack;
		std::stack<Handle> soln_handle_stack;

		// Stacks containing partial groundings.
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> var_solutn_stack;
		std::stack<SolnMap> term_solutn_stack;

		std::stack<IssuedSet> issued_stack;
		std::stack<ChoiceState> choice_stack;

		// push, pop and clear these states.
		void solution_push(void);
		void solution_pop(void);
		void clause_stacks_push(void);
		void clause_stacks_pop(void);
		void clause_stacks_clear(void);
		unsigned int _clause_stack_depth;

		// -------------------------------------------
		// Recursive tree comparison algorithm.
		typedef enum {
			CALL_QUOTE,
			CALL_ORDER,
			CALL_UNORDER,
			CALL_CHOICE,
			CALL_COMP,
			CALL_SOLN
		} Caller;   // temporary scaffolding !???

		bool tree_compare(const Handle&, const Handle&, Caller);
		bool tree_recurse(const Handle&, const Handle&, Caller);
		bool redex_compare(const LinkPtr&, const LinkPtr&);

		// See PatternMatchEngine.cc for descriptions
		bool start_sol_up(const Handle&);
		bool xsoln_up(const Handle&);
		bool do_term_up(const Handle&);
		bool clause_accept(const Handle&);
		bool do_next_clause(void);

		bool clause_accepted;
		void get_next_untried_clause(void);
		bool get_next_untried_helper(bool, bool, bool);

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

	public:
		PatternMatchEngine(PatternMatchCallback&,
		                   const Variables&,
		                   const Pattern&);

		// Examine the locally connected neighborhood for possible
		// matches.
		bool explore_neighborhood(const Handle&, const Handle&, const Handle&);

		// Handy-dandy utilities
		static void print_solution(const std::map<Handle, Handle> &vars,
		                           const std::map<Handle, Handle> &clauses);

		static void print_term(const std::set<Handle> &vars,
		                            const std::vector<Handle> &clauses);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_ENGINE_H
