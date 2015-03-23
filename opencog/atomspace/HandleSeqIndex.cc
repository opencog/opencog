/*
 * opencog/atomspace/HandleSeqIndex.cc
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

#include <opencog/atomspace/HandleSeqIndex.h>

using namespace opencog;

void HandleSeqIndex::insert(const HandleSeq &seq, Handle h)
{
	idx.insert(std::pair<const HandleSeq,Handle>(seq,h));
}

Handle HandleSeqIndex::get(const HandleSeq &seq) const
{
	std::map<const HandleSeq,Handle>::const_iterator it;

	it = idx.find(seq);
	if (it != idx.end()) return it->second;

	return Handle::UNDEFINED;
}

void HandleSeqIndex::remove(const HandleSeq &seq, Handle h)
{
	idx.erase(seq);
}

size_t HandleSeqIndex::size(void) const
{
	return idx.size();
}

void HandleSeqIndex::remove(bool (*filter)(Handle))
{
	std::map<const HandleSeq,Handle>::iterator i, j;
	
	i = idx.begin();
	while (i != idx.end())
	{
		j = i;
		++i;
		if (filter(j->second))
			idx.erase(j->first);
	}
}

