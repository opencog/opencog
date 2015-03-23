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

using namespace opencog;

void FixedIntegerIndex::insert(int i, Handle h)
{
	UnorderedUUIDSet &s = idx.at(i);
	s.insert(h.value());
}

Handle FixedIntegerIndex::get(int i) const
{
	return Handle::UNDEFINED;
}

void FixedIntegerIndex::remove(int i, Handle h)
{
	UnorderedUUIDSet &s = idx.at(i);
	s.erase(h.value());
}

size_t FixedIntegerIndex::size(void) const
{
	size_t cnt = 0;
	std::vector<UnorderedUUIDSet >::const_iterator s;
	for (s = idx.begin(); s != idx.end(); ++s)
	{
		cnt += s->size();
	}
	return cnt;
}

void FixedIntegerIndex::remove(bool (*filter)(Handle))
{
	std::vector<UnorderedUUIDSet >::iterator s;
	for (s = idx.begin(); s != idx.end(); ++s)
	{
		UnorderedUUIDSet::iterator i, j;
	
		i = s->begin();
		while (i != s->end())
		{
			j = i;
			++i;
			if (filter(Handle(*j)))
				s->erase(*j);
		}
	}
}

void FixedIntegerIndex::resize(size_t sz)
{
	idx.resize(sz);
}

// ================================================================
