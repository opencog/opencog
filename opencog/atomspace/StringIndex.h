/*
 * opencog/atomspace/StringIndex.h
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

#ifndef _OPENCOG_STRINGINDEX_H
#define _OPENCOG_STRINGINDEX_H

#include <cstddef>
#include <map>

#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Implements map from string to atom pointers. Used to implement
 * the lookup of atoms according to thier name.
 */
class StringIndex
{
	private:
		std::map<std::string, Atom*> idx;

	public:
		void insert(const std::string& str, Atom* a)
		{
			idx.insert(std::pair<std::string, Atom*>(str, a));
		}
		Atom* get(const std::string& str) const
		{
			std::map<std::string, Atom*>::const_iterator it;
			it = idx.find(str);
			if (it != idx.end()) return it->second;
			return NULL;
		}
		void remove(const std::string& str)
		{
			idx.erase(str);
		}
		size_t size(void) const
		{
			return idx.size();
		}
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_STRINGINDEX_H
