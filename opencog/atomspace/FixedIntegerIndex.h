/*
 * opencog/atomspace/FixedIntegerIndex.h
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

#ifndef _OPENCOG_FIXEDINTEGERINDEX_H
#define _OPENCOG_FIXEDINTEGERINDEX_H

#include <cstddef>
#include <vector>

#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef std::unordered_set<int> UnorderedIntSet;

/**
 * Implements an integer index as an RB-tree (C++ map)
 */
class FixedIntegerIndex
{
	protected:
		std::vector<UnorderedIntSet> idx;
		void resize(size_t);

	public:
		~FixedIntegerIndex() {}
		void insert(int, Handle);
		Handle get(int) const;
		void remove(int, Handle);
		size_t size(void) const;
		void remove(bool (*)(Handle));

		size_t size(int i) const
		{
			const UnorderedIntSet &s = idx.at(i);
			return s.size();
		}
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_FIXEDINTEGERINDEX_H
