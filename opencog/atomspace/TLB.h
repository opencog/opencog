/*
 * opencog/atomspace/TLB.h
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
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

#ifndef _OPENCOG_TLB_H
#define _OPENCOG_TLB_H

#include <mutex>
#include <unordered_map>

#include <opencog/util/Logger.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>

class TLBUTest;
class BasicSaveUTest;

namespace opencog
{

class AtomSpaceBenchmark;
class AtomStorage;
class AtomTable;

/**
 * Each atom stored on OpenCog will have an immutable UUID, which will be used
 * to refer to that atom when a reference to that atom needs to be kept.
 * Each proxy must have a look-up mechanism or table (TLB) to map from
 * this ID to the actual memory address for the atom in the local process
 * address space.
 */
class TLB
{
    friend class Atom;
    friend class AtomSpaceBenchmark;
    friend class AtomStorage;
    friend class AtomTable;
    friend class ::TLBUTest;
    friend class ::BasicSaveUTest;

private:

    // Single, global mutex for locking the TLB.
    static std::mutex _mtx;

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

    static UUID _brk_uuid;

    /** Adds a new atom to the TLB.
     * If the atom has already be added then an exception is thrown.
     *
     * @param Atom to be added.
     * @return Handle of the newly added atom.
     */
    static inline void addAtom(AtomPtr atom);

    static inline bool isInvalidHandle(const Handle& h);

    static inline bool isValidHandle(const Handle& h);

    static UUID getMaxUUID(void) { return _brk_uuid; }
    static inline void reserve_range(UUID lo, UUID hi)
    {
        std::lock_guard<std::mutex> lck(_mtx);
        if (_brk_uuid <= hi) _brk_uuid = hi+1;
    }
};

inline bool TLB::isInvalidHandle(const Handle& h)
{
    return (h == Handle::UNDEFINED) ||
           (h.value() >= _brk_uuid) || 
           (NULL == h);  // Hmmm this is not right. XXX
           // We could have a non-zero uuid, but the pointer is null,
           // because the atom is in storage or a remote server.
}

inline bool TLB::isValidHandle(const Handle& h)
{
    return not isInvalidHandle(h);
}

inline void TLB::addAtom(AtomPtr atom)
{
    std::lock_guard<std::mutex> lck(_mtx);

    if (atom->_uuid != Handle::UNDEFINED.value()) {
        throw InvalidParamException(TRACE_INFO,
        "Atom is already in the TLB!");
    }

    atom->_uuid = _brk_uuid;
    _brk_uuid++;
}

} // namespace opencog

#endif // _OPENCOG_TLB_H
