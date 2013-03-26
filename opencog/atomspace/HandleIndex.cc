/*
 * opencog/atomspace/HandleIndex.cc
 *
 * Copyright (C) 2008,2009,2013 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/util/oc_assert.h>
#include <opencog/atomspace/HandleIndex.h>

using namespace opencog;

void HandleIndex::insert(Handle h, const HandleSeq &seq)
{
	idx.insert(std::pair<Handle, const HandleSeq>(h, seq));
}

const HandleSeq& HandleIndex::get(Handle h) const
{
	std::map<Handle, const HandleSeq>::const_iterator it;

	it = idx.find(h);
	if (it != idx.end()) return it->second;

	static HandleSeq empty;
	return empty;
}

void HandleIndex::remove(Handle h, const HandleSeq &seq)
{
	idx.erase(h);
}

size_t HandleIndex::size(void) const
{
	return idx.size();
}

void HandleIndex::remove(bool (*filter)(const HandleSeq&))
{
	OC_ASSERT(0, "Unexpected call to unimplemented function!");
}

void HandleIndex::remove(bool (*filter)(Handle))
{
	std::map<Handle, const HandleSeq>::iterator i, j;
	
	i = idx.begin();
	while (i != idx.end())
	{
		j = i;
		++i;
		if (filter(j->first))
			idx.erase(j->first);
	}
}

