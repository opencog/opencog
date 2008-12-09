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

#include <set>
#include <vector>

#include <opencog/atomspace/AtomIndex.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
class HandleEntry;

/**
 * Implements an integer index as an RB-tree (C++ map)
 */
class FixedIntegerIndex:
	public AtomIndex<int,Handle>
{
	private:
		std::vector<std::set<Handle> > idx;

	public:
		virtual void insert(int, Handle);
		virtual Handle get(int) const;
		virtual void remove(int, Handle);
		virtual size_t size(void) const;
		virtual void remove(bool (*)(Handle));

		void resize(size_t);
		HandleEntry* getHandleSet(Type type, bool subclass) const;

		class iterator
		{
			friend class FixedIntegerIndex;
			public:
				iterator(Type, bool);
				iterator& operator++(int);
				iterator& operator=(iterator);
				bool operator==(iterator);
				bool operator!=(iterator);
				Handle operator*(void);
			private:
				Type type;
				bool subclass;
				std::vector<std::set<Handle> >::const_iterator s;
				std::vector<std::set<Handle> >::const_iterator send;
				Type currtype;
				std::set<Handle>::const_iterator se;
		};

		iterator begin(Type, bool) const;
		iterator end(void) const;
};

} //namespace opencog

#endif // _OPENCOG_FIXEDINTEGERINDEX_H
