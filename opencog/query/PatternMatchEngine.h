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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog {

class PatternMatchEngine
{
	// Private, locally scoped typedefs, not used outside of this class.
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList *> RootMap;
	typedef std::pair<Handle, RootList *> RootPair;

	protected:
		AtomSpace *atom_space;

	private:
		bool prt(Handle& h);
		void prtmsg(const char *msg, Handle& h);

		// -------------------------------------------
		// predicate to be solved.
		std::set<Handle> _bound_vars;
		std::vector<Handle> _cnf_clauses;
		std::set<Handle> _optionals;

		// -------------------------------------------
		// Traversal utilities
		RootMap root_map;
		Handle curr_root;
		bool note_root(Handle);
		
		// -------------------------------------------
		// Recurisve tree comparison algorithm.
		bool tree_compare(Handle, Handle);
		int depth;  // recursion depth for tree_compare.

		bool pred_up(Handle);
		bool soln_up(Handle);
		bool do_soln_up(Handle&);
		bool clause_accepted;
		Handle curr_soln_handle;
		Handle curr_pred_handle;
		void get_next_untried_clause(void);

		// Stack used during recursive exploration
		std::stack<Handle> pred_handle_stack;
		std::stack<Handle> soln_handle_stack;
		std::stack<Handle> root_handle_stack;

		// Stacks containing partial groundings.
		typedef std::map<Handle, Handle> SolnMap;
		std::stack<SolnMap> pred_solutn_stack;
		std::stack<SolnMap> var_solutn_stack;

		// Set of clauses for which a grounding is currently being attempted.
		typedef std::set<Handle> IssuedSet;
		IssuedSet issued;
		std::stack<IssuedSet> issued_stack;

		// -------------------------------------------

		// Result of solving the predicate
		std::map<Handle, Handle> var_grounding;
		std::map<Handle, Handle> clause_grounding;

		// Handle used to denote non-existant grounding.
		Handle invalid_grounding;

		// callback to report results.
		PatternMatchCallback *pmc;

	public:
		PatternMatchEngine(void);
		void set_atomspace(AtomSpace *);
		AtomSpace * get_atomspace(void) { return atom_space; } 

		// Clear all internal state
		void clear(void);

		// Examine each candidate for a match, in turn.
		bool do_candidate(Handle&, Handle&, Handle&);

		// Make sure that variables can be found in the clauses.
		bool validate(const std::set<Handle> &vars,
		              std::vector<Handle> &clauses);

		bool validate(const std::set<Handle> &vars,
		              Handle& clause);

		void get_connected_components(const std::set<Handle> &vars,
		              const std::vector<Handle> &clauses,
		              std::set<std::vector<Handle>> &components);

		// Do the actual pattern search.
		void match(PatternMatchCallback *,
		           std::set<Handle> &vars,
		           std::vector<Handle> &clauses,
		           std::vector<Handle> &negations);

		// Handy-dandy utilities
		void print_solution(const std::map<Handle, Handle> &vars,
		                           const std::map<Handle, Handle> &clauses);

		void print_predicate(const std::set<Handle> &vars,
		                            const std::vector<Handle> &clauses);
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_ENGINE_H
