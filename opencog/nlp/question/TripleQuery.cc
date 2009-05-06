/**
 * TripleQuery.cc
 *
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
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

#include "TripleQuery.h"

#include <stdio.h>

using namespace opencog;

TripleQuery::TripleQuery(void)
{
}

TripleQuery::~TripleQuery()
{
}

/* ======================================================== */

void TripleQuery::clear(void)
{
	bound_vars.clear();
	normed_predicate.clear();
	if (pme) delete pme;
	pme = NULL;
}

void TripleQuery::add_triple(Handle h)
{
	add_to_predicate(h);
}

void TripleQuery::solve(AtomSpace *as)
{
	// Find the variables, so that they can be bound.
	bound_vars.clear();
	std::vector<Handle>::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		Handle h = *i;
		find_vars(h);
	}

#define DEBUG
#ifdef DEBUG
	PatternMatchEngine::print_predicate(bound_vars, normed_predicate);
#endif

	// Some bad relex parses fail to actually have query variables
	// in them. If there are no variables, don't bother looking for
	// a solution.  XXX Except that this won't actually work, because
	// yes/no questions will have no query variables, e.g. Did X do Y?
	// if (0 == bound_vars.size()) return;

	atom_space = as;
	if (pme) delete pme;
	pme = new PatternMatchEngine();
	pme->set_atomspace(atom_space);

	// Solve...
	std::vector<Handle> ign;
	pme->match(this, bound_vars, normed_predicate, ign);
}

/* ===================== END OF FILE ===================== */
