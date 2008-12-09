/*
 * opencog/atomspace/FixedIntegerIndex.cc
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

#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/type_codes.h>
#include <opencog/atomspace/HandleEntry.h>

using namespace opencog;

void FixedIntegerIndex::insert(int i, Handle h)
{
	std::set<Handle> &s = idx.at(i);
	s.insert(h);
}

Handle FixedIntegerIndex::get(int i) const
{
	return Handle::UNDEFINED;
}

void FixedIntegerIndex::remove(int i, Handle h)
{
	std::set<Handle> &s = idx.at(i);
	s.erase(h);
}

size_t FixedIntegerIndex::size(void) const
{
	size_t cnt = 0;
	std::vector<std::set<Handle> >::const_iterator s;
	for (s = idx.begin(); s != idx.end(); s++)
	{
		cnt += s->size();
	}
	return cnt;
}

void FixedIntegerIndex::remove(bool (*filter)(Handle))
{
	std::vector<std::set<Handle> >::iterator s;
	for (s = idx.begin(); s != idx.end(); s++)
	{
		std::set<Handle>::iterator i, j;
	
		i = s->begin();
		while (i != s->end())
		{
			j = i;
			i++;
			if (filter(*j))
				s->erase(*j);
		}
	}
}

void FixedIntegerIndex::resize(size_t sz)
{
	idx.resize(sz);
}

// ================================================================

HandleEntry * FixedIntegerIndex::getHandleSet(Type type, bool subclass) const
{
	iterator it = begin(type, subclass);
	iterator itend = end();

printf("duude enter handle set ty=%d sub=%d\n", type, subclass);
	HandleEntry *he = NULL;
	while (it != itend)
	{
printf("duuude her we are, handle=%lu\n", (*it).value());
		HandleEntry *nhe = new HandleEntry(*it);
		nhe->next = he;
		he = nhe;
		it++;
	}
	return he;
}

// ================================================================

FixedIntegerIndex::iterator FixedIntegerIndex::begin(Type t, bool sub) const
{
	iterator it(t, sub);
	it.send = idx.end();

	it.s = idx.begin();
	it.currtype = 0;
	while (it.s != idx.end())
	{
		// Find the first type which is a subtype, and start iteration there.
		if ((it.type == it.currtype) || 
		    (sub && (ClassServer::isAssignableFrom(it.type, it.currtype))))
		{
			it.se = it.s->begin();
			if (it.se != it.s->end()) return it;
		}
		it.currtype++;
		it.s++;
	}

	return it;
}

FixedIntegerIndex::iterator FixedIntegerIndex::end(void) const
{
	iterator it(ATOM, false);
	it.se = idx.at(ATOM).end();
	it.s = idx.end();
	it.send = idx.end();
printf ("duuuude the END !!\n");
	return it;
}

FixedIntegerIndex::iterator::iterator(Type t, bool sub)
{
	type = t;
	subclass = sub;
}

FixedIntegerIndex::iterator& FixedIntegerIndex::iterator::operator=(iterator v)
{
	s = v.s;
	send = v.send;
	se = v.se;
	return *this;
}

Handle FixedIntegerIndex::iterator::operator*(void)
{
	return *se;
}

bool FixedIntegerIndex::iterator::operator==(iterator v)
{
	if ((v.s == v.send) && (s == send)) return true;
	return v.se == se;
}

bool FixedIntegerIndex::iterator::operator!=(iterator v)
{
printf ("compare %d and %d\n", v.s==v.send, s==send);
	if ((v.s == v.send) && (s != send)) return v.se != se;
	if ((v.s != v.send) && (s == send)) return v.se != se;
	return false;
}

FixedIntegerIndex::iterator& FixedIntegerIndex::iterator::operator++(int i)
{
printf ("duude increment %d\n", s==send);
	if (s == send) return *this;

	se++;
	if (se == s->end())
	{
		do
		{
			s++;
			currtype++;

			// Find the first type which is a subtype, and start iteration there.
			if ((type == currtype) || 
			    (subclass && (ClassServer::isAssignableFrom(type, currtype))))
			{
				se = s->begin();
				if (se != s->end()) return *this;
			}
		} while (s != send);
	}

	return *this;
}

// ================================================================
