/**
 * TLB.h
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef TLB_H
#define TLB_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define USE_TLB_MAP
#ifdef USE_TLB_MAP
#include <map>
#endif

#include "Atom.h" 
#include "types.h" 

class Atom;

/**
 * Each atom stored on OpenCog will have an immutable ID, which will be used
 * to refer to that atom when a reference to that atom needs to be kept.
 * Each proxy must have a look-up mechanism or table (TLB) to map from 
 * this ID to the actual memory address for the atom in the local process
 * address sapce.
 */
class TLB {

    private:

        static unsigned long uuid;
        static std::map<Handle, Atom *> handle_map;
        static std::map<Atom *, Handle> atom_map;

        /**
         * Private default constructor for this class to make it abstract.
         */
        TLB() {}

    public:

#define OBFUSCATE 0x55555555
// #define OBFUSCATE 0x0
        /**
         * Maps a handle to its corresponding atom.
         *
         * @param Handle to be mapped.
         * @return Corresponding atom for the given handle.
         */
        static inline Atom* getAtom(Handle handle)
        {
#ifdef USE_TLB_MAP
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
        static inline Handle getHandle(const Atom* atom)
        {
#ifdef USE_TLB_MAP
            Handle h = atom_map[(Atom *) atom];
            if (h != 0) return h;
            return addAtom(atom);
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
        static inline Handle addAtom(const Atom* atom)
        {
#ifdef USE_TLB_MAP
            Atom *a = (Atom *) atom;
            Handle h = atom_map[a];
            if (h != 0)
            {
                // throw InvalidParamException(TRACE_INFO, "Atom is already in the TLB");
                return h;
            }
            h = uuid;
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
            Handle h = atom_map[atom];
            atom_map.erase(atom);
            handle_map.erase(h);
            return atom;
#else
            return atom;
#endif
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

#endif
