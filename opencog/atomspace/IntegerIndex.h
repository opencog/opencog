/*
 * opencog/atomspace/IntegerIndex.h
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

#ifndef _OPENCOG_INTEGERINDEX_H
#define _OPENCOG_INTEGERINDEX_H

#include <map>

#include <opencog/atomspace/AtomIndex.h>
#include <opencog/atomspace/Handle.h>

namespace opencog
{

/**
 * Implements an integer index as an RB-tree (C++ map)
 */
class IntegerIndex:
	public AtomIndex<int,Handle>
{
	private:
		std::map<int, Handle> idx;

	public:
		virtual void insert(int, Handle);
		virtual Handle get(int) const;
		virtual void remove(int);
		virtual size_t size(void) const;
		virtual void remove(bool (*)(int, Handle));
};

} //namespace opencog

#endif // _OPENCOG_INTEGERINDEX_H
