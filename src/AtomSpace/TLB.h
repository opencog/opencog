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

#include <stdio.h>
#include <string.h>
#include "Atom.h" 
#include "types.h" 

class Atom;

/**
 * Each atom stored on MindDB will have an immutable ID, which will be used
 * to refer to that atom when a reference to that atom needs to be kept.
 * Each proxy must have a look-up mechanism or table (TLB) to map from 
 * this ID to the actual memory address for the atom in the local process
 * address sapce.
 */
class TLB {

    private:

        /**
         * Private default constructor for this class to make it abstract.
         */
        TLB() {}

    public:

        /**
         * Maps a handle to its corresponding atom.
         *
         * @param Handle to be mapped.
         * @return Corresponding atom for the given handle.
         */
        static inline Atom* getAtom(Handle handle) {
            return handle;
        }

        /**
         * Maps an atom to its corresponding handle.
         *
         * @param Atom to be mapped.
         * @return Corresponding handle for the given atom.
         */
        static inline Handle getHandle(const Atom* atom) {
            return (Handle) atom;
        }

        /**
         * Adds a new atom to the TLB.
         *
         * @param Atom to be added.
         * @return Handle of the newly added atom.
         */
        static inline Handle addAtom(Atom* atom) {
            return atom;
        }

        /**
         * Removes an atom from the TLB.
         *
         * @param Atom to be removed.
         * @return Removed atom.
         */
        static inline Atom* removeAtom(Atom* atom) {
            return atom;
        }        
};

#endif
