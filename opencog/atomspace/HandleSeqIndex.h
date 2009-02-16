/*
 * opencog/atomspace/HandleSeqIndex.h
 *
 * Copyright (C) 2008,2009 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_HANDLE_SEQ_INDEX_H
#define _OPENCOG_HANDLE_SEQ_INDEX_H

#include <map>

#include <opencog/atomspace/AtomIndex.h>
#include <opencog/atomspace/Handle.h>

namespace opencog
{

/**
 * Implements a Handle-sequence index as an RB-tree (C++ map)
 */
class HandleSeqIndex:
	public AtomIndex<HandleSeq,Handle>
{
	private:
		std::map<HandleSeq, Handle> idx;

	public:
		virtual void insert(HandleSeq, Handle);
		virtual Handle get(HandleSeq) const;
		virtual void remove(HandleSeq, Handle);
		virtual size_t size(void) const;
		virtual void remove(bool (*)(Handle));
};

} //namespace opencog

#endif // _OPENCOG_HANDLE_SEQ_INDEX_H
