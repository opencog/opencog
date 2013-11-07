/*
 * opencog/atomspace/NodeIndex.cc
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

#include <opencog/atomspace/NodeIndex.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/atom_types.h>

using namespace opencog;

NodeIndex::NodeIndex()
{
	resize();
}

void NodeIndex::resize()
{
	this->idx.resize(classserver().getNumberOfClasses());
}

size_t NodeIndex::size() const
{
	size_t cnt = 0;
	size_t vsz = idx.size();
	for (size_t i=0; i<vsz; i++) cnt += idx[i].size();
	return cnt;
}

void NodeIndex::insertAtom(AtomPtr a)
{
	Type t = a->getType();
	NameIndex &ni = idx[t];
	ni.insertAtom(a);
}

void NodeIndex::removeAtom(AtomPtr a)
{
	Type t = a->getType();
	NameIndex &ni = idx[t];
	ni.removeAtom(a);
}

Handle NodeIndex::getHandle(Type t, const char *name) const
{
	if (t >= idx.size()) throw RuntimeException(TRACE_INFO,
		   "Index out of bounds for atom type (t = %lu)", t);
	const NameIndex &ni = idx[t];
	return ni.get(name);
}

void NodeIndex::remove(bool (*filter)(Handle))
{
	std::vector<NameIndex>::iterator s;
	for (s = idx.begin(); s != idx.end(); ++s) {
		s->remove(filter);
	}
}

UnorderedHandleSet NodeIndex::getHandleSet(Type type, const char *name,
		bool subclass) const
{
	UnorderedHandleSet hs;
	if (subclass) {
		
		int max = classserver().getNumberOfClasses();
		for (Type s = 0; s < max; s++) {
			// The 'AssignableFrom' direction is unit-tested in
			// AtomSpaceUTest.cxxtest
			if (classserver().isA(s, type)) {
				if (s >= idx.size()) throw RuntimeException(TRACE_INFO,
						  "Index out of bounds for atom type (s = %lu)", s);
				const NameIndex &ni = idx[s];
				Handle h = ni.get(name);
				if (Handle::UNDEFINED != h)
					hs.insert(h);
			}
		}
	} else {
		Handle h = getHandle(type, name);
		if (Handle::UNDEFINED != h) hs.insert(h);
	}

	return hs;
}

// ================================================================
