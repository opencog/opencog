/*
 * opencog/atomspace/StringIndex.cc
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

#include <opencog/atomspace/StringIndex.h>

using namespace opencog;

/*
 * XXX I suspect that using std:string for the key leads to
 * a highly inefficient search, with lots of string copying 
 * going on. Just a guess; I dunno. It would be a *LOT* more
 * storage efficient if opencog used a string cache, so that
 * atom names would not be duplicated here, and also in the
 * atom itself. But that's a project for another day.
 */
void StringIndex::insert(const char * str, Handle h)
{
    // Don't add unnamed handles
    // if (str == NULL || *str == 0)
    // Err, well, no ... Due to some sort of crazy-shit arguments
    // with Trent Waddington, he insisted that this was possible and 
    // required. So he added support, but incompletely. And then
    // he didn't actually test his code to see if it worked. So of
    // course it didn't. And it leaked memory. Whoops. Which I had 
    // to debug. So now we allow atoms with null string names to
    // be added to the atom space. WTF.  But now his tests fail.
    // Go figure. I don't understand.
    if (str == NULL)
        return;
	idx.insert(std::pair<std::string,Handle>(str,h));
}

Handle StringIndex::get(const char *str) const
{
	std::map<std::string,Handle>::const_iterator it;

	it = idx.find(str);
	if (it != idx.end()) return it->second;

	return Handle::UNDEFINED;
}

void StringIndex::remove(const char *str, Handle h)
{
	idx.erase(str);
}

size_t StringIndex::size(void) const
{
	return idx.size();
}

void StringIndex::remove(bool (*filter)(Handle))
{
	std::map<std::string,Handle>::iterator i, j;
	
	i = idx.begin();
	while (i != idx.end())
	{
		j = i;
		++i;
		if (filter(j->second))
			idx.erase(j->first.c_str());
	}
}

