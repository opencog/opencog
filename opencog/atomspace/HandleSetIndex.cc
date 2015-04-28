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
#include <opencog/atomspace/HandleSetIndex.h>

using namespace opencog;

void HandleSetIndex::remove(bool (*filter)(const UnorderedHandleSet&))
{
	OC_ASSERT(0, "Unexpected call to unimplemented function!");
}

void HandleSetIndex::remove(bool (*filter)(Handle))
{
	std::map<Handle, const UnorderedHandleSet>::iterator i, j;
	
	i = idx.begin();
	while (i != idx.end())
	{
		j = i;
		++i;
		Handle h = j->first;
		if (filter(h))
			idx.erase(h);
	}
}
