/*
 * opencog/atomspace/HandleIndex.h
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

#ifndef _OPENCOG_HANDLE_SET_INDEX_H
#define _OPENCOG_HANDLE_SET_INDEX_H

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
 * Implements a Handle index as an RB-tree (C++ map)
 * That is, given a Handle, it will return an UnorderedHandleSet
 * associated with that Handle.  This index is the "opposite" to the 
 * HandleSeqIndex; the map goes in the opposite direction.
 */
class HandleSetIndex:
	public AtomIndex<Handle, const UnorderedHandleSet&>
{
	private:
		std::map<Handle, const UnorderedHandleSet> idx;

	public:
		virtual void insert(Handle, const UnorderedHandleSet&);
		virtual const UnorderedHandleSet& get(Handle) const;
		virtual void remove(Handle, const UnorderedHandleSet&);
		virtual size_t size(void) const;
		virtual void remove(bool (*)(const UnorderedHandleSet&));
		void remove(bool (*)(Handle));
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_HANDLE_SET_INDEX_H
