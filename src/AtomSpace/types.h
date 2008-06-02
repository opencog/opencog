/*
 * src/AtomSpace/types.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

/**
 * basic type definitions.
 */

#ifndef OPENCOG_TYPES_H
#define OPENCOG_TYPES_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>
#include "platform.h"

class Atom;

typedef opencog::ConstCharPointerIntHashMap ClassTypeHashMap;
typedef opencog::IntConstCharPointerHashMap ClassNameHashMap;

// Definition of a handle. Opaque type.
// Will change when system is reworked for distributed computing.
typedef unsigned long Handle;

//#ifdef WIN32
//typedef hash_map<Handle, void *> HandleVoidPointerHashMap;
//#else
struct hashHandle {
    int operator()(Handle h) const;
};

struct eqHandle {
    bool operator()(Handle h1, Handle h2) const;
};

typedef opencog::hash_map<Handle, void *, hashHandle, eqHandle> HandleVoidPointerHashMap;
//#endif

// type and arity of Atoms, represented as short integers (16 bits)
typedef unsigned short Type;
typedef unsigned short Arity;

// This was a 16-bit representation of a float, which has access methods implemented by
// the ShortFloatOps class. It has changed to a float but in the future can be brought back
// to a smaller representation
typedef float ShortFloat;


/**
 * This class provides basic operations over the ShortFloat type.
 */
class ShortFloatOps
{

private:

    /**
     * Private default constructor for this class to make it abstract.
     */
    ShortFloatOps() {}

public:

    /**
     * Maps ShortFloat to float.
     *
     * @return Float conversion of a ShortFloat.
     */
    static float getValue(const ShortFloat*);

    /**
     * Maps float to ShortFloat.
     *
     * @param ShortFloat location to receive value.
     * @param Float value to be set.
     */
    static void setValue(ShortFloat*, float);
};

/**
 * Structure used to return a linked-list of atoms instead of the standard
 * linked-list of handles (HandleEntry)
 */
struct AtomEntry {
    AtomEntry *next;
    Atom *atom;
};

#endif /* OPENCOG_TYPES_H */
