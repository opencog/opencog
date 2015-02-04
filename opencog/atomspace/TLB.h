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

#include <atomic>

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

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

    // Thread-safe atomic
    static std::atomic<UUID> _brk_uuid;

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

    /// Reserve a range of UUID's.  The range is inclusive; both lo and
    /// hi are reserved.  The range must NOT intersect with the
    /// currently issued UUID's.
    static inline void reserve_range(UUID lo, UUID hi)
    {
        if (hi <= lo)
            throw InvalidParamException(TRACE_INFO,
                "Bad argument order.");
        UUID extent = hi - lo + 1;

        UUID oldlo = _brk_uuid.fetch_add(extent, std::memory_order_relaxed);

        if (lo < oldlo)
            throw InvalidParamException(TRACE_INFO,
                "Bad range reserve.");
    }

    /// Reserve an extent of UUID's. The lowest reserved ID is returned.
    /// That is, after this call, no one else will be issued UUID's in
    /// the range of [retval, retval+extent-1].
    static inline UUID reserve_extent(UUID extent)
    {
        return _brk_uuid.fetch_add(extent, std::memory_order_relaxed);
    }
};

inline bool TLB::isInvalidHandle(const Handle& h)
{
    return (h == Handle::UNDEFINED) ||
           (h.value() >= _brk_uuid);
}

inline bool TLB::isValidHandle(const Handle& h)
{
    return not isInvalidHandle(h);
}

inline void TLB::addAtom(AtomPtr atom)
{
    if (atom->_uuid != Handle::UNDEFINED.value())
        throw InvalidParamException(TRACE_INFO,
                "Atom is already in the TLB!");

    atom->_uuid = _brk_uuid.fetch_add(1, std::memory_order_relaxed);
}

} // namespace opencog

#endif // _OPENCOG_TLB_H
