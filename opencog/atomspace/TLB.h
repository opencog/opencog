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
#ifdef HAVE_SQL_STORAGE
#define USE_TLB_MAP 1
#endif

#define CHECK_MAP_CONSISTENCY

#ifdef USE_TLB_MAP
#include <map>
#endif

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/type_codes.h>
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
    static std::map<Handle, const Atom*> handle_map;
    static std::map<const Atom *, Handle> atom_map;
#endif

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

#ifdef USE_TLB_MAP
    static unsigned long uuid;
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
        if (handle.value() <= NOTYPE) // check for "non-real" atoms
            return reinterpret_cast<Atom*>(handle.value());

        std::map<Handle, const Atom*>::iterator it = handle_map.find(handle);
        if (it == handle_map.end()) return NULL;
        else return const_cast<Atom*>(it->second);
#else
        return reinterpret_cast<Atom*>(handle.value() ^ OBFUSCATE);
#endif
    }

    /**
     * Maps an atom to its corresponding handle.
     *
     * @param Atom to be mapped.
     * @return Corresponding handle for the given atom.
     */
    static inline Handle getHandle(const Atom* atom)
    {
#ifdef USE_TLB_MAP
        Handle h(reinterpret_cast<unsigned long>(atom));
        if (h.value() <= NOTYPE) return h; // check for "non-real" atoms

        std::map<const Atom*, Handle>::iterator it = atom_map.find(atom);
        if (it != atom_map.end()) return it->second;
#ifdef CHECK_MAP_CONSISTENCY
        throw InvalidParamException(TRACE_INFO,
            "Atom is not in the TLB!");
        return Handle::UNDEFINED;
#else
        return addAtom(atom);
#endif
#else
        return Handle(reinterpret_cast<unsigned long>(atom) ^ OBFUSCATE);
#endif
    }

    /**
     * Maps an atom to its corresponding handle. This method should
     * only be used to check if the atom is already in the TLB; and
     * this check should only be performed prior to adding to the TLB.
     * For all other cases, use the getHandle call above.
     *
     * The difference between this and the TLB::getHandle() method 
     * is that getHandle() will throw if the atom is not in the TLB,
     * whereas this mathod simply returns Handle::UNDEFINED.
     *
     * @param Atom to be mapped.
     * @return Corresponding handle for the given atom.
     */
    static inline Handle holdsHandle(const Atom* atom)
    {
#ifdef USE_TLB_MAP
        Handle h(reinterpret_cast<unsigned long>(atom));
        if (h.value() <= NOTYPE) return h; // check for "non-real" atoms

        std::map<const Atom*, Handle>::iterator it = atom_map.find(atom);
        if (it != atom_map.end()) return it->second;
        return Handle::UNDEFINED;
#else
        return Handle(reinterpret_cast<unsigned long>(atom) ^ OBFUSCATE);
#endif
    }

    /**
     * Adds a new atom to the TLB.
     *
     * @param Atom to be added.
     * @return Handle of the newly added atom.
     */
    static inline Handle addAtom(const Atom* atom,
                                 Handle handle = Handle::UNDEFINED)
    {
#ifdef USE_TLB_MAP
        Handle h(reinterpret_cast<unsigned long>(atom));
        if (h.value() <= NOTYPE) return h; // check for "non-real" atoms

        std::map<const Atom*, Handle>::iterator it = atom_map.find(atom);
        if (it != atom_map.end())
        {
#ifdef CHECK_MAP_CONSISTENCY
            throw InvalidParamException(TRACE_INFO,
            "Atom is already in the TLB!");
#endif /* CHECK_MAP_CONSISTENCY */
            /* Hmm, I guess its okay to add an atom twice, assuming
             * that it is being added with the same handle. */
            if (handle != it->second)
                throw InvalidParamException(TRACE_INFO,
                "Atom is already in the TLB with a different handle!");
            return it->second;
        }
        if (handle == Handle::UNDEFINED) handle = Handle(uuid);
        handle_map[handle] = atom;
        atom_map[atom] = handle;
        uuid++;
        return handle;
#else /* USE_TLB_MAP */
        return Handle(reinterpret_cast<unsigned long>(atom) ^ OBFUSCATE);
#endif /* USE_TLB_MAP */
    }

    /**
     * Removes an atom from the TLB.
     *
     * @param Atom to be removed.
     * @return Removed atom.
     */
    static inline const Atom* removeAtom(const Atom* atom) {
#ifdef USE_TLB_MAP
        Handle h(reinterpret_cast<unsigned long>(atom));
        if (h.value() <= NOTYPE) return atom; // check for "non-real" atoms

        std::map<const Atom*, Handle>::iterator it = atom_map.find(atom);
        if (it == atom_map.end()) {
#ifdef CHECK_MAP_CONSISTENCY
            throw InvalidParamException(TRACE_INFO,
                "Cannot remove: Atom is not in the TLB");
#endif
            return atom;
        }
        atom_map.erase(atom);
        handle_map.erase(it->second);
#endif
        return atom;
    }

    static inline bool isInvalidHandle(const Handle& h) {
#ifdef USE_TLB_MAP
        return (h == Handle::UNDEFINED) || (h.value() >= uuid);
#else
        return (h == Handle::UNDEFINED);
#endif
    }

    static inline bool isValidHandle(Handle h) {
#ifdef USE_TLB_MAP
        return (h != Handle::UNDEFINED) && (h.value() < uuid);
#else
        return (h != Handle::UNDEFINED);
#endif
    }

#ifdef USE_TLB_MAP
    static unsigned long getMaxUUID(void) { return uuid; }
    static void reserve_range(unsigned long lo, unsigned long hi)
    {
        if (uuid <= hi) uuid = hi+1;
    }
#endif
};

} // namespace opencog

#endif // _OPENCOG_TLB_H
