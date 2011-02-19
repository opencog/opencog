/*
 * opencog/atomspace/TLB.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Linas Vepstas <linasvepstas@gmail.com>
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

#define CHECK_MAP_CONSISTENCY

#include <boost/unordered_map.hpp>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>

#define OBFUSCATE (0x55555555UL)

class AtomSpaceUTest;
class AtomSpaceImplUTest;
class AtomTableUTest;
class LinkUTest;
class NodeUTest;
class CompositeTruthValueUTest;
class HandleEntryUTest;
class HandleSetUTest;
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
    friend class AtomSpaceImpl;
    friend class ::AtomSpaceUTest;
    friend class ::AtomSpaceImplUTest;
    friend class AtomTable;
    friend class ::AtomTableUTest;

    // TODO review these AtomSpace friend classes to see whether they
    // are allowed to access the TLB in the way that they do.
    friend class Atom;
    friend class Node;
    friend class ::NodeUTest;
    friend class Link;
    friend class ::LinkUTest;
    friend class HandleEntry;
    friend class ::HandleEntryUTest;
    friend class HandleSet;
    friend class ::HandleSetUTest;
    friend class HandleTemporalPair;
    friend class HandleToTemporalEntryMap;
    friend class ImportanceIndex;
    friend class LinkIndex;
    friend class NameIndex;
    friend class NodeIndex;
    friend class PredicateIndex;
    friend class SpaceServer;
    friend class TargetTypeIndex;
    friend class Trail;
    friend class TypeIndex;
    friend class ::TemporalTableUTest;
    friend class ::TimeServerUTest;
    friend class ::CompositeTruthValueUTest;
    friend class AtomSpaceBenchmark;

    // TODO work out if TLB can be removed from these persistance
    // related classes
    friend class NMXmlExporter;
    friend class ::BasicSCMUTest;
    friend class CoreUtils;
    friend class SavingLoading;
    friend class ::BasicSaveUTest;
    friend class SpaceServerSavable;
    friend class TemporalTableFile;
    friend class AtomStorage;
    friend class SenseSimilaritySQL;

private:

    static boost::unordered_map<Handle, const Atom*, boost::hash<opencog::Handle> > handle_map;

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
    static inline Atom* getAtom(const Handle& handle)
    {
        boost::unordered_map<Handle, const Atom*>::iterator it = handle_map.find(handle);
        if (it == handle_map.end()) return NULL;
        else return const_cast<Atom*>(it->second);
    }

    /**
     * Adds a new atom to the TLB.
     *
     * If the atom has already be added then an exception is thrown.
     *
     * @param Atom to be added.
     * @return Handle of the newly added atom.
     */
    static inline const Handle& addAtom(Atom* atom,
                                 const Handle &handle = Handle::UNDEFINED)
    {
        const Handle &h = atom->handle;
        if (h != Handle::UNDEFINED)
        {
#ifdef CHECK_MAP_CONSISTENCY
            throw InvalidParamException(TRACE_INFO,
            "Atom is already in the TLB!");
#endif /* CHECK_MAP_CONSISTENCY */
            /* Hmm, I guess its okay to add an atom twice, assuming
             * that it is being added with the same handle. */
            if (handle != h)
                throw InvalidParamException(TRACE_INFO,
                "Atom is already in the TLB with a different handle!");
            return h;
        }

        Handle ha = handle;
        if (ha == Handle::UNDEFINED)
        {
            ha = Handle(brk_uuid);
            brk_uuid++;
        }
        handle_map[ha] = atom;
        atom->handle = ha;
        return atom->handle;
    }

    /**
     * Removes an atom from the TLB.
     *
     * If the atom has already been removed from or never been in the TLB
     * then an exception is thrown.
     *
     * @param Atom to be removed.
     * @return Removed atom.
     */
    static inline const Atom* removeAtom(Atom* atom) {
        const Handle &h = atom->handle;
        if (h == Handle::UNDEFINED) {
#ifdef CHECK_MAP_CONSISTENCY
            throw InvalidParamException(TRACE_INFO,
                "Cannot remove: Atom is not in the TLB");
#endif
            return atom;
        }
        handle_map.erase(h);
        atom->handle = Handle::UNDEFINED;
        return atom;
    }

    static inline bool isInvalidHandle(const Handle& h) {
        return (h == Handle::UNDEFINED) ||
               (h.value() >= brk_uuid) || 
               (NULL == getAtom(h));
    }

    static inline bool isValidHandle(Handle h) {
        return !isInvalidHandle(h);
    }

    static UUID getMaxUUID(void) { return brk_uuid; }
    static void reserve_range(UUID lo, UUID hi)
    {
        if (brk_uuid <= hi) brk_uuid = hi+1;
    }
};

} // namespace opencog

#endif // _OPENCOG_TLB_H
