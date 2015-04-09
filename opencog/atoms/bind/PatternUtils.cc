/*
 * PatternUtils.cc
 *
 * Copyright (C) 2008,2009,2011,2014 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  February 2008
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

#include <opencog/atomutils/FindUtils.h>
#include "PatternUtils.h"

using namespace opencog;

namespace opencog {

/**
 * Remove constant clauses from the list of clauses.
 *
 * Make sure that every clause contains at least one variable;
 * if not, remove the clause from the list of clauses.
 *
 * The core idea is that pattern matching against a constant expression
 * "doesn't make sense" -- the constant expression will always match to
 * itself and is thus "trivial".  In principle, the programmer should
 * never include constants in the list of clauses ... but, due to
 * programmer error, this can happen, and will lead to failures during
 * pattern matching. Thus, the routine below can be used to validate
 * the input.
 *
 * Terms that contain GroundedSchema or GroundedPrecdicate nodes can
 * have side-effects, and are thus not really constants. They must be
 * evaluated during the pattern search.
 *
 * Returns true if the list of clauses was modified, else returns false.
 */
bool remove_constants(const std::set<Handle> &vars,
                      std::vector<Handle> &clauses)
{
	bool modified = false;

	// Caution: this loop modifies the clauses list!
	std::vector<Handle>::iterator i;
	for (i = clauses.begin(); i != clauses.end(); )
	{
		Handle clause(*i);
		if (any_unquoted_in_tree(clause, vars)
		    or contains_atomtype(clause, GROUNDED_PREDICATE_NODE)
		    or contains_atomtype(clause, GROUNDED_SCHEMA_NODE))
		{
			++i;
		}
		else
		{
			i = clauses.erase(i);
			modified = true;
		}
	}

	return modified;
}

/* ======================================================== */
/**
 * Given an input set of clauses, partition this set into its
 * connected components, returning a list of the connected components,
 * and a matching list of the variables that appear in each component.
 *
 * Two clauses are "connected" if they both contain a common
 * variable. A connected component is the set of all clauses that are
 * connected to one-another, in some way.
 *
 * This method serves two different purposes.  First, if the
 * pattern does not contain any "virtual" links, then the pattern
 * matcher works correctly only if there is one single, connected
 * component (this is by design, since we don't want to deal with
 * the combinatoric explosion of grounding multuple distinct components).
 * If the pattern does contain "virtual" links, then the connected
 * components should be grounded first, and the results then combined
 * by exploring the combinatoric possibilites presented by the virtual
 * link(s).
 *
 * A side effect of the algorithm is that it sorts the clauses into
 * connected order. That is, given the vector of connected clauses,
 * each element in the vector is connected to some clause that came
 * before it (in the vector).  This is handy, because it guarantees
 * that the next clause must be connected to the previous ones. This
 * speeds up the discovery of the next ungrounded clause: it is
 * trivially just the very next clause in the connected set.  Of
 * course, users will typically never specify clauses in such order.
 */
void get_connected_components(const std::set<Handle>& vars,
                              const HandleSeq& clauses,
                              std::vector<HandleSeq>& components,
                              std::vector<std::set<Handle>>& component_vars)
{
	std::vector<Handle> todo(clauses);

	while (0 < todo.size())
	{
		// no_con_yet == clauses that failed to connect to any existing
		// component.
		std::vector<Handle> no_con_yet;
		bool did_at_least_one = false;

		for (const Handle& cl: todo)
		{
			bool extended = false;

			// Which component might this possibly belong to??? Try them all.
			size_t nc = components.size();
			for (size_t i = 0; i<nc; i++)
			{
				std::set<Handle>& cur_vars(component_vars[i]);
				// If clause cl is connected to this component, then add it
				// to this component.
				if (any_unquoted_in_tree(cl, cur_vars))
				{
					// Extend the component
					components[i].push_back(cl);

					// Add to the varset cache for that component.
					FindAtoms fv(vars);
					fv.search_set(cl);
					for (const Handle& v : fv.varset) cur_vars.insert(v);

					extended = true;
					did_at_least_one = true;
					break;
				}
			}

			if (not extended)
				no_con_yet.push_back(cl);
		}

		if (did_at_least_one)
		{
			todo = no_con_yet;
			continue;
		}

		// Grab the first clause that failed to attach to something,
		// and use it to start a new component.
		Handle ncl(no_con_yet.back());
		no_con_yet.pop_back();
		todo = no_con_yet;

		// If we are here, we found a disconnected clause.
		// Start a new component
		components.push_back({ncl});

		FindAtoms fv(vars);
		fv.search_set(ncl);
		component_vars.push_back(fv.varset);
	}
}

} // namespace opencog

/* ===================== END OF FILE ===================== */
