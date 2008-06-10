/*
 * src/AtomSpace/TLB.h
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

#ifndef TLB_H
#define TLB_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

// Use the TLB map only if SQL storage is being used.
#ifdef HAVE_SQL_STORAGE
#define USE_TLB_MAP 1
#endif

#define CHECK_MAP_CONSISTENCY

#ifdef USE_TLB_MAP
#include <map>
#endif

#include "Atom.h"
#include "types.h"
#include "type_codes.h"

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
    static std::map<Handle, Atom *> handle_map;
    static std::map<Atom *, Handle> atom_map;
#endif

    /**
     * Private default constructor for this class to make it abstract.
     */
    TLB() {}

public:
#ifdef USE_TLB_MAP
    static unsigned long uuid;
#endif

#define OBFUSCATE 0x55555555
// #define OBFUSCATE 0x0
    /**
     * Maps a handle to its corresponding atom.
     *
     * @param Handle to be mapped.
     * @return Corresponding atom for the given handle.
     */
    static inline Atom* getAtom(Handle handle) {
#ifdef USE_TLB_MAP
        if (handle <= NOTYPE) return (Atom *) handle; // check for "non-real" atoms
        return handle_map[handle];
#else
        return (Atom *) (handle ^ OBFUSCATE);
#endif
    }

    /**
     * Maps an atom to its corresponding handle.
     *
     * @param Atom to be mapped.
     * @return Corresponding handle for the given atom.
     */
    static inline Handle getHandle(const Atom* atom) {
#ifdef USE_TLB_MAP
        Handle h = (Handle) atom;
        if (h <= NOTYPE) return h; // check for "non-real" atoms

        h = atom_map[(Atom *) atom];
        if (h != 0) return h;
#ifdef CHECK_MAP_CONSISTENCY
        return UndefinedHandle();
#else
        return addAtom(atom);
#endif
#else
        return ((Handle) atom) ^ OBFUSCATE;
#endif
    }

    /**
     * Adds a new atom to the TLB.
     *
     * @param Atom to be added.
     * @return Handle of the newly added atom.
     */
    static inline Handle addAtom(const Atom* atom, Handle h = 0) {
#ifdef USE_TLB_MAP
        Handle ht = (Handle) atom;
        if (ht <= NOTYPE) return ht; // check for "non-real" atoms

        Atom *a = (Atom *) atom;
        Handle ha = atom_map[a];
        if (ha != 0) {
#ifdef CHECK_MAP_CONSISTENCY
            if (h != ha) throw InvalidParamException(TRACE_INFO, "Atom is already in the TLB");
#endif
            return ha;
        }
        if (h == 0) h = uuid;
        handle_map[h] = a;
        atom_map[a] = h;
        uuid++;
        return h;
#else
        return ((Handle) atom) ^ OBFUSCATE;
#endif
    }

    /**
     * Removes an atom from the TLB.
     *
     * @param Atom to be removed.
     * @return Removed atom.
     */
    static inline Atom* removeAtom(Atom* atom) {
#ifdef USE_TLB_MAP
        Handle h = (Handle) atom;
        if (h <= NOTYPE) return atom; // check for "non-real" atoms

        h = atom_map[atom];
        if (h == 0) {
#ifdef CHECK_MAP_CONSISTENCY
            throw InvalidParamException(TRACE_INFO, "Atom is not in the TLB");
#endif
            return atom;
        }
        atom_map.erase(atom);
        handle_map.erase(h);
#endif
        return atom;
    }

    static inline bool isInvalidHandle(Handle h) {
#ifdef USE_TLB_MAP
        return (h == 0) || (h >= uuid);
#else
        return (h == OBFUSCATE);
#endif
    }

    static inline bool isValidHandle(Handle h) {
#ifdef USE_TLB_MAP
        return (h != 0) && (h < uuid);
#else
        return (h != OBFUSCATE);
#endif
    }

    static inline Handle UndefinedHandle(void) {
#ifdef USE_TLB_MAP
        return 0;
#else
        return OBFUSCATE;
#endif
    }

#define UNDEFINED_HANDLE (TLB::UndefinedHandle())
};

} // namespace opencog

#endif
