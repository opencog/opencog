/*
 * opencog/atomspace/HandleIterator.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef _OPENCOG_HANDLE_ITERATOR_H
#define _OPENCOG_HANDLE_ITERATOR_H

#include <opencog/atomspace/atom_types.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/TypeIndex.h>
#include <opencog/atomspace/VersionHandle.h>

namespace opencog {

class AtomTable;

/**
 * This class provides an iterator that cycles through atoms in the AtomTable
 * according to specific criteria.
 * XXX TODO eliminate this class entirely, ASAP
 */
class HandleIterator
{
    friend class AtomTable;

private:
    TypeIndex::iterator it;
    TypeIndex::iterator end;

    /**
     * Internal constructor that initializes an iterator for atoms of a
     * given type (subclasses optionally).
     *
     * @param Atom type to be iterated.
     * @param Whether the above type should consider subclasses as well.
     */
    HandleIterator(AtomTable *, Type type = ATOM, bool subclass = false);

public:

    /**
     * Returns whether there still are atoms to be iterated.
     *
     * @return Whether there still are atoms to be iterated.
     */
    bool hasNext(void) { return it != end; }

    /**
     * Returns the next atom of the iterator and advances.
     *
     * @return Next atom of the iterator and advances.
     */
    Handle next(void) { Handle h = *it; it++; return h; }
};

} // namespace opencog

#endif // _OPENCOG_HANDLE_ITERATOR_H
