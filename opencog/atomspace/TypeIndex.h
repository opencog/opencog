/*
 * opencog/atomspace/TypeIndex.h
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

#ifndef _OPENCOG_TYPEINDEX_H
#define _OPENCOG_TYPEINDEX_H

#include <set>
#include <vector>

#include <opencog/atomspace/FixedIntegerIndex.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
class HandleEntry;

/**
 * Implements an integer index as an RB-tree (C++ set)
 */
class TypeIndex:
	public FixedIntegerIndex
{
	private:
		size_t num_types;
	public:
		TypeIndex(void);
		void insertAtom(const Atom*);
		void removeAtom(const Atom*);
		void resize(void);

		HandleEntry* getHandleSet(Type type, bool subclass) const;

		class iterator
		{
			friend class TypeIndex;
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
				std::vector<UnorderedHandleSet>::const_iterator s;
				std::vector<UnorderedHandleSet>::const_iterator send;
				Type currtype;
				UnorderedHandleSet::const_iterator se;
		};

		iterator begin(Type, bool) const;
		iterator end(void) const;
};

} //namespace opencog

#endif // _OPENCOG_TYPEINDEX_H
