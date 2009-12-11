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

// Use the TLB map only if SQL storage is being used.
// (or if distributed processing is enabled)
// Various tests fail if TLB is not enabled. Until these are fixed,
// we will enable the TLB. Anyway, long-term, it is needed, so may
// as well turn it on for good ... 
// #ifdef HAVE_SQL_STORAGE
#define USE_TLB_MAP 1
// #endif

#define CHECK_MAP_CONSISTENCY

#ifdef USE_TLB_MAP
#include <boost/unordered_map.hpp>
#endif

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>

#define OBFUSCATE (0x55555555UL)

namespace opencog
{

/**
 * Each atom stored on OpenCog will have an immutable ID, which will be used
 * to refer to that atom when a reference to that atom needs to be kept.
 * Each proxy must have a look-up mechanism or table (TLB) to map from
 * this ID to the actual memory address for the atom in the local process
 * address space.
 */
class TLB
{
private:

#ifdef USE_TLB_MAP
    static boost::unordered_map<Handle, const Atom*, boost::hash<opencog::Handle> > handle_map;
#endif

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

#ifdef USE_TLB_MAP
    static UUID brk_uuid;
#endif

public:
    /**
     * Maps a handle to its corresponding atom.
     *
     * @param Handle to be mapped.
     * @return Corresponding atom for the given handle. Returns NULL if handle
     * isn't found.
     */
    static inline Atom* getAtom(const Handle& handle)
    {
#ifdef USE_TLB_MAP
        boost::unordered_map<Handle, const Atom*>::iterator it = handle_map.find(handle);
        if (it == handle_map.end()) return NULL;
        else return const_cast<Atom*>(it->second);
#else
        if (Handle::UNDEFINED == handle) return NULL;
        return reinterpret_cast<Atom*>(handle.value() ^ OBFUSCATE);
#endif
    }

    /**
     * Maps an atom to its corresponding handle.
     *
     * If the atom is not in the TLB an exception is thrown.
     *
     * @param Atom to be mapped.
     * @return Corresponding handle for the given atom.
     */
    static inline const Handle& getHandle(const Atom* atom)
    {
#ifdef USE_TLB_MAP
        const Handle &h = atom->handle;
        if (h != Handle::UNDEFINED) return h;
#ifdef CHECK_MAP_CONSISTENCY
        throw InvalidParamException(TRACE_INFO,
                                    "Atom is not in the TLB!");
        return Handle::UNDEFINED;
#else
        return addAtom(atom);
#endif
#else
        return Handle(reinterpret_cast<UUID>(atom) ^ OBFUSCATE);
#endif
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
#ifdef USE_TLB_MAP
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
#else /* USE_TLB_MAP */
        return Handle(reinterpret_cast<UUID>(atom) ^ OBFUSCATE);
#endif /* USE_TLB_MAP */
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
#ifdef USE_TLB_MAP

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
#endif
        return atom;
    }

    static inline bool isInvalidHandle(const Handle& h) {
#ifdef USE_TLB_MAP
        return (h == Handle::UNDEFINED) ||
               (h.value() >= brk_uuid) || 
               (NULL == getAtom(h));
#else
        return (h == Handle::UNDEFINED);
#endif
    }

    static inline bool isValidHandle(Handle h) {
        return !isInvalidHandle(h);
    }

#ifdef USE_TLB_MAP
    static UUID getMaxUUID(void) { return brk_uuid; }
    static void reserve_range(UUID lo, UUID hi)
    {
        if (brk_uuid <= hi) brk_uuid = hi+1;
    }
#endif
};

} // namespace opencog

#endif // _OPENCOG_TLB_H
