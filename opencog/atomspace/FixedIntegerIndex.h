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

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef std::unordered_set<AtomPtr> UnorderedAtomSet;

/**
 * Implements a vector of atom sets; each set can be found via an
 * integer index.
 */
class FixedIntegerIndex
{
	protected:
		std::vector<UnorderedAtomSet> idx;
		void resize(size_t sz)
		{
			idx.resize(sz);
		}

	public:
		~FixedIntegerIndex() {}
		void insert(size_t i, AtomPtr a)
		{
			UnorderedAtomSet &s(idx.at(i));
			s.insert(a);
		}

		void remove(size_t i, AtomPtr a)
		{
			UnorderedAtomSet &s = idx.at(i);
			s.erase(a);
		}

		size_t size(size_t i) const
		{
			const UnorderedAtomSet &s(idx.at(i));
			return s.size();
		}

		size_t size(void) const;
		void remove(bool (*)(Handle));
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_FIXEDINTEGERINDEX_H
