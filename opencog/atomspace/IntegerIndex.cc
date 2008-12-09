/*
 * opencog/atomspace/IntegerIndex.cc
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

#include <opencog/atomspace/IntegerIndex.h>

using namespace opencog;

void IntegerIndex::insert(int i, Handle h)
{
	idx.insert(std::pair<int,Handle>(i,h));
}

Handle IntegerIndex::get(int i) const
{
	std::map<int,Handle>::const_iterator it;

	it = idx.find(i);
	if (it != idx.end()) return it->second;

	return Handle::UNDEFINED;
}

void IntegerIndex::remove(int i)
{
	idx.erase(i);
}

size_t IntegerIndex::size(void) const
{
	return idx.size();
}

void IntegerIndex::remove(bool (*filter)(Handle))
{
	std::map<int,Handle>::iterator i, j;
	
	i = idx.begin();
	while (i != idx.end())
	{
		j = i;
		i++;
		if (filter(j->second))
			idx.erase(j->first);
	}
}

