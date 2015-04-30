/*
 * embodiment/AtomSpaceExtensions/GetByOutgoing.h
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef _OPENCOG_GET_OUTGOING_H
#define _OPENCOG_GET_OUTGOING_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

using namespace opencog;

     /**
     * Returns the set of atoms with the given target handles and types
     * (order is considered) in their outgoing sets, where the type and
     * subclasses of the atoms are optional.
     *
     * @param  An array of handles to match the outgoing sets of the
     *         searched atoms. This array can be empty (or each of its
     *         elements can be null), if the handle value does not
     *         matter or if it does not apply to the specific search.
     *         Note that if this array is not empty, it must contain
     *         "arity" elements.
     * @param  An array of target types to match the types of the atoms
     *         in the outgoing set of searched atoms.
     * @param  An array of boolean values indicating whether each of the
     *         above types must also consider subclasses. This array can
     *         be null, which means that subclasses will not be
     *         considered.
     *         Note that if this array is not null, it must contains
     *         "arity" elements.
     * @param  The length of the outgoing set of the atoms being
     * searched.
     * @param  The optional type of the atom.
     * @param  Whether atom type subclasses should be considered.
     * @return The set of atoms of the given type with the matching
     *         criteria in their outgoing set.
     *
     * @note The matched entries are appended to a container whose
     *       OutputIterator is passed as the first argument. Example
     *       of call to this method, which would return all entries
     *       in AtomSpace:
     * @code
     *     std::list<Handle> ret;
     *     atomSpace.getHandleSet(back_inserter(ret), ATOM, true);
     * @endcode
     */
    UnorderedHandleSet getHandlesByOutgoingSet(
                 AtomSpace&,
                 Type type,
                 const HandleSeq& handles,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 bool subclass = false);

    template <typename OutputIterator> OutputIterator
    getHandlesByOutgoing(OutputIterator result,
                 AtomSpace& as,
                 Type type,
                 const HandleSeq& handles,
                 Type* types,
                 bool* subclasses,
                 Arity arity,
                 bool subclass = false)
    {
        UnorderedHandleSet hs = getHandlesByOutgoingSet(
                as, type, handles,
                types, subclasses, arity, subclass);
        return std::copy(hs.begin(), hs.end(), result);
    }

#endif
