/*
 * opencog/atomspace/TargetTypeIndex.h
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

#ifndef _OPENCOG_TARGETINDEX_H
#define _OPENCOG_TARGETINDEX_H

#include <set>
#include <vector>

#include <opencog/atomspace/TypeIndex.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class Link;

/**
 * Implements an integer index as an RB-tree (C++ set)
 */
class TargetTypeIndex:
	public TypeIndex
{
	public:
		TargetTypeIndex(void);
		void insertAtom(AtomPtr);
		void removeAtom(AtomPtr);
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_TARGETINDEX_H
