/*
 * opencog/atomspace/AtomIndex.h
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

#ifndef _OPENCOG_ATOMINDEX_H
#define _OPENCOG_ATOMINDEX_H

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * This class provides a simple, generic interface for maintaining 
 * arbitrary indexes of Atoms. Indexes are used whenever a system needs
 * rapid lookup of atoms having some particular type or property.
 *
 * A secondary goal of this interface class is to allow different
 * implementations to use the same interface: implementations as
 * linked-lists, rb-trees, hash tables, etc. This flexibility is 
 * important for choosing a RAM/CPU performance profile customized for
 * the index.
 *
 * Typically, Value will be Handle, possibly PredicateEvaluator*.
 * The Key will typically be an int, string, etc.
 */

template <typename Key, typename Value>
class AtomIndex
{
	public:
		virtual ~AtomIndex() {}
		virtual void insert(Key, Value) = 0;
		virtual Value get(Key) const = 0;
		virtual void remove(Key, Value) = 0;
		virtual size_t size(void) const = 0;
		virtual void remove(bool (*)(Value)) = 0;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATOMINDEX_H
