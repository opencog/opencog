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
	idx.resize(classserver().getNumberOfClasses());
}

size_t NodeIndex::size() const
{
	size_t cnt = 0;
	for (const NameIndex& ni : idx) cnt += ni.size();
	return cnt;
}

void NodeIndex::remove(bool (*filter)(AtomPtr))
{
	for (NameIndex ni : idx) ni.remove(filter);
}

UnorderedHandleSet NodeIndex::getHandleSet(Type type, const std::string& name,
		bool subclass) const
{
	UnorderedHandleSet hs;
	if (subclass) {

		Type max = classserver().getNumberOfClasses();
		for (Type s = 0; s < max; s++) {
			// The 'AssignableFrom' direction is unit-tested in
			// AtomSpaceUTest.cxxtest
			if (classserver().isA(s, type)) {
				AtomPtr atom(getAtom(type, name));
				if (atom)
					hs.insert(atom->getHandle());
			}
		}
	} else {
		AtomPtr atom(getAtom(type, name));
		if (atom) hs.insert(atom->getHandle());
	}

	return hs;
}

// ================================================================
