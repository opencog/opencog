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

#include <unordered_map>

#include <opencog/util/Logger.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/types.h>

// WTF this is like, total bullshit!  Who did this? 
// Get rid of these UTest class crap ...
class TLBUTest;
class AtomSpaceUTest;
class AtomTableUTest;
class LinkUTest;
class NodeUTest;
class CompositeTruthValueUTest;
class HandleEntry;
class HandleEntryUTest;
class TemporalTableUTest;
class TimeServerUTest;
class BasicSaveUTest;
class BasicSCMUTest;
class AtomSpaceBenchmark;

namespace opencog
{

/**
 * Each atom stored on OpenCog will have an immutable UUID, which will be used
 * to refer to that atom when a reference to that atom needs to be kept.
 * Each proxy must have a look-up mechanism or table (TLB) to map from
 * this ID to the actual memory address for the atom in the local process
 * address space.
 */
class TLB
{
    friend class ::AtomSpaceUTest;
    friend class Atom;
    friend class AtomTable;
    friend class ::AtomTableUTest;
    friend class ::TLBUTest;

    // TODO review these AtomSpace friend classes to see whether they
    // are allowed to access the TLB in the way that they do.
    friend class ::NodeUTest;
    friend class ::LinkUTest;
    friend class ::HandleEntry;
    friend class ::HandleEntryUTest;
    friend class HandleTemporalPair;
    friend class HandleToTemporalEntryMap;
    friend class ImportanceIndex;
    friend class LinkIndex;
    friend class NameIndex;
    friend class NodeIndex;
    friend class PredicateIndex;
    friend class TargetTypeIndex;
    friend class Trail;
    friend class TypeIndex;
    friend class ::TemporalTableUTest;
    friend class ::TimeServerUTest;
    friend class ::CompositeTruthValueUTest;
    friend class AtomSpaceBenchmark;

    // TODO work out if TLB can be removed from these persistance
    // related classes
    friend class ::BasicSCMUTest;
    friend class CoreUtils;
    friend class ::BasicSaveUTest;
    friend class AtomStorage;
    friend class SenseSimilaritySQL;

    typedef std::unordered_map< Handle, AtomPtr, handle_hash > map_t;
private:

    static map_t handle_map;

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

    static UUID brk_uuid;

    /**
     * Maps a handle to its corresponding atom.
     *
     * @param Handle to be mapped.
     * @return Corresponding atom for the given handle. Returns NULL if handle
     * isn't found.
     */
    static inline AtomPtr getAtom(const Handle& handle);

    /** Adds a new atom to the TLB.
     * If the atom has already be added then an exception is thrown.
     *
     * @param Atom to be added.
     * @return Handle of the newly added atom.
     */
    static inline const Handle& addAtom(AtomPtr atom,
                                 const Handle &handle = Handle::UNDEFINED);

    /**
     * Removes an atom from the TLB.
     *
     * If the atom has already been removed from or never been in the TLB
     * then an exception is thrown.
     *
     * @param handle of atom to be removed.
     * @return Removed atom.
     */
    static inline AtomPtr removeAtom(const Handle& h);

    static inline bool isInvalidHandle(const Handle& h);

    static inline bool isValidHandle(const Handle& h);

    static UUID getMaxUUID(void) { return brk_uuid; }
    static inline void reserve_range(UUID lo, UUID hi)
    {
        if (brk_uuid <= hi) brk_uuid = hi+1;
    }

    static void print();
};

inline bool TLB::isInvalidHandle(const Handle& h)
{
    return (h == Handle::UNDEFINED) ||
           (h.value() >= brk_uuid) || 
           (NULL == getAtom(h));
}

inline bool TLB::isValidHandle(const Handle& h)
{
    return !isInvalidHandle(h);
}

inline const Handle& TLB::addAtom(AtomPtr atom, const Handle &handle)
{
    const Handle &h = atom->handle;
    if (h != Handle::UNDEFINED) {
        throw InvalidParamException(TRACE_INFO,
        "Atom is already in the TLB!");
    }

    Handle ha = handle;
    if (ha == Handle::UNDEFINED) {
        ha = Handle(brk_uuid);
        brk_uuid++;
    }
    handle_map[ha] = atom;
    atom->handle = ha;
    return atom->handle;
}

inline AtomPtr TLB::getAtom(const Handle& handle)
{
    map_t::iterator it = handle_map.find(handle);
    if (it == handle_map.end()) return NULL;
    else return it->second;
}

inline AtomPtr TLB::removeAtom(const Handle& h)
{
    if (h == Handle::UNDEFINED) return NULL;

    AtomPtr atom = TLB::getAtom(h);
    if (NULL == atom) return NULL;

    // Remove from the map
    handle_map.erase(h);
    // blank the old handle so it is clear this Atom is no longer
    // in the TLB
    atom->handle = Handle::UNDEFINED;
    return atom;
}

} // namespace opencog

#endif // _OPENCOG_TLB_H
