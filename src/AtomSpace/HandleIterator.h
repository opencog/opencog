/*
 * src/AtomSpace/HandleIterator.h
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

#ifndef HANDLEITERATOR_H
#define HANDLEITERATOR_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "classes.h"
#include "types.h"
#include "VersionHandle.h"

/**
 * This class provides an iterator that cycles through atoms in the AtomTable
 * according to specific criteria.
 */

class AtomTable;

class HandleIterator
{

    friend class AtomTable;

private:

    AtomTable* table;

    Type currentType;

    Handle currentHandle;

    Type desiredType;

    bool desiredTypeSubclass;

    VersionHandle desiredVersionHandle;

    /**
     * Internal method that initializes an iterator for atoms of a given
     * type (subclasses optionally).
     *
     * @param Atom type to be iterated.
     * @param Whether the above type should consider subclasses as well.
     * @param VersionHandle for filtering the resulting atoms by context. NULL_VERSION_HANDLE indicates no filtering
     */
    void init(AtomTable *, Type, bool, VersionHandle);

    /**
     * Internal constructor that initializes an iterator for atoms of a
     * given type (subclasses optionally).
     *
     * @param Atom type to be iterated.
     * @param Whether the above type should consider subclasses as well.
     * @param VersionHandle for filtering the resulting atoms by context. NULL_VERSION_HANDLE indicates no filtering
     *
     */
    HandleIterator(AtomTable *, Type type = ATOM, bool subclass = false, VersionHandle vh = NULL_VERSION_HANDLE);

public:

    /**
     * Destructor for this class.
     */
    ~HandleIterator();

    /**
     * Returns whether there still are atoms to be iterated.
     *
     * @return Whether there still are atoms to be iterated.
     */
    bool hasNext();

    /**
     * Returns the next atom of the iterator and advances.
     *
     * @return Next atom of the iterator and advances.
     */
    Handle next();
};

#endif
