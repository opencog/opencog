/*
 * opencog/atomspace/TypeIndex.cc
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

#include "TypeIndex.h"
#include "Atom.h"
#include "ClassServer.h"

using namespace opencog;

TypeIndex::TypeIndex(void)
{
	resize();
}

void TypeIndex::resize(void)
{
	num_types = classserver().getNumberOfClasses();
	FixedIntegerIndex::resize(num_types + 1);
}

void TypeIndex::insertAtom(AtomPtr a)
{
	Type t = a->getType();
	insert(t, a->getHandle());
}

void TypeIndex::removeAtom(AtomPtr a)
{
	Type t = a->getType();
	remove(t, a->getHandle());
}

// ================================================================

TypeIndex::iterator TypeIndex::begin(Type t, bool sub) const
{
	iterator it(t, sub);
	it.send = idx.end();

	it.s = idx.begin();
	it.currtype = 0;
	while (it.s != it.send)
	{
		// Find the first type which is a subtype, and start iteration there.
		if ((it.type == it.currtype) || 
		    (sub && (classserver().isA(it.currtype, it.type))))
		{
			it.se = it.s->begin();
			if (it.se != it.s->end()) return it;
		}
		it.currtype++;
		++it.s;
	}

	return it;
}

TypeIndex::iterator TypeIndex::end(void) const
{
	iterator it(num_types, false);
	it.se = idx.at(num_types).end();
	it.s = idx.end();
	it.send = idx.end();
	it.currtype = num_types;
	return it;
}

TypeIndex::iterator::iterator(Type t, bool sub)
{
	type = t;
	subclass = sub;
}

TypeIndex::iterator& TypeIndex::iterator::operator=(iterator v)
{
	s = v.s;
	send = v.send;
	se = v.se;
	currtype = v.currtype;
	type = v.type;
	subclass = v.subclass;
	return *this;
}

Handle TypeIndex::iterator::operator*(void)
{
	if (s == send) return Handle::UNDEFINED;
	return Handle(*se);
}

bool TypeIndex::iterator::operator==(iterator v)
{
	if ((v.s == v.send) && (s == send)) return true;
	return v.se == se;
}

bool TypeIndex::iterator::operator!=(iterator v)
{
	if ((v.s == v.send) && (s != send)) return v.se != se;
	if ((v.s != v.send) && (s == send)) return v.se != se;
	return false;
}

TypeIndex::iterator& TypeIndex::iterator::operator++()
{
	return operator++(1);
}

TypeIndex::iterator& TypeIndex::iterator::operator++(int i)
{
	if (s == send) return *this;

	++se;
	if (se == s->end())
	{
		do
		{
			++s;
			currtype++;

			// Find the first type which is a subtype, and start iteration there.
			if ((type == currtype) || 
			    (subclass && (classserver().isA(currtype, type))))
			{
				se = s->begin();
				if (se != s->end()) return *this;
			}
		} while (s != send);
	}

	return *this;
}

// ================================================================
