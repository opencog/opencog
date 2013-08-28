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

#include <opencog/atomspace/AtomIndex.h>
#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Implements a string index as an RB-tree (C++ map)
 */
class StringIndex:
	public AtomIndex<const char *, Handle>
{
	private:
		std::map<std::string, Handle> idx;

	public:
		virtual void insert(const char *, Handle);
		virtual Handle get(const char *) const;
		virtual void remove(const char *, Handle);
		virtual size_t size(void) const;
		virtual void remove(bool (*)(Handle));
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_STRINGINDEX_H
