/*
 * opencog/atomspace/NameIndex.h
 *
 * Copyright (C) 2008,2015 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_NAMEINDEX_H
#define _OPENCOG_NAMEINDEX_H

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StringIndex.h>
#include <opencog/atomspace/types.h>

namespace opencog
{

/**
 * Implements an atom name index as an RB-tree (C++ map)
 * That is, given an atom name, this returns an AtomPtr to that atom.
 * This class can only hold *one* AtomPtr for a given name. For a
 * multi-name storage, use the NodeIndex, which keys by name and
 * atom type.
 */
class NameIndex:
	public StringIndex
{
	public:
		void insertAtom(AtomPtr a)
		{
			NodePtr n(NodeCast(a));
			if (NULL == n) return;
			insert(n->getName(), a);
		}
		void removeAtom(AtomPtr a)
		{
			NodePtr n(NodeCast(a));
			if (NULL == n) return;
			remove(n->getName());
		}
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_NAMEINDEX_H
