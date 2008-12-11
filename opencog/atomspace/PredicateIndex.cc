/*
 * opencog/atomspace/PredicateIndex.cc
 *
 * Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/atomspace/PredicateIndex.h>
#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/HandleEntry.h>

using namespace opencog;

PredicateIndex::PredicateIndex(void)
{
	resize(MAX_PREDICATE_INDICES);
}

void PredicateIndex::removeHandle(Handle h)
{
	std::vector<std::set<Handle> >::iterator s;
	for (s = idx.begin(); s != idx.end(); s++)
	{
		s->erase(h);
	}
}

HandleEntry * PredicateIndex::getHandleSet(int index) const
{
	const std::set<Handle> &s = idx.at(index);
	std::set<Handle>::const_iterator it;
	HandleEntry *he = NULL;
	for (it = s.begin(); it != s.end(); it++)
	{
		HandleEntry *nhe = new HandleEntry(*it);
		nhe->next = he;
		he = nhe;
	}
	return he;
}
// ================================================================
