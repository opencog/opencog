/*
 * opencog/atomspace/HandleSet.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Rodrigo Barra
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

#ifndef _OPENCOG_HANDLE_SET_H
#define _OPENCOG_HANDLE_SET_H

#include <opencog/atomspace/types.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

namespace opencog
{

class HandleSetIterator;

class HandleSet
{
    friend class HandleSetIterator;

private:
    /**
     * Defines a hash set used to store handles.
     */
    typedef UnorderedHandleSet InternalHandleSet;

public:

    /**
     * Defines an iterator to the handleSetp.
     */
    typedef InternalHandleSet::iterator InternalIterator;

private:

    /**
     * The handleSet where the elements will be stored.
     */
    InternalHandleSet *handleSet;

    /**
     * Constructor used by clone.
     */
    HandleSet(InternalHandleSet *);

public:

    /**
     * Constructor for this class.
     */
    HandleSet();

    /**
     * Destructor for this class
     */
    ~HandleSet();


    /**
     * Returns a copy of a HandleSet.
     */
    HandleSet *clone();

    /**
     * Adds a new entry to the handle set.
     *
     * @param Key.
     */
    void add(Handle);

    /**
     * Adds the content of another HandleSet into the handle set.
     *
     * @param HandleSet.
     */
    void add(HandleSet *);

    /**
     * Checks if there exists an element for the given key.
     *
     * @param Key.
     */
    bool contains(Handle) const;

    /**
     * Removes an element referred by a given key from the set.
     *
     * @param Key.
     */
    void remove(Handle) throw (RuntimeException);

    /**
     * Returns the total number of elements in the hash set.
     *
     * @return Total number of elements in the hash set.
     */
    int getSize();

    /**
     * Returns an iterator through all Handles stored in the handle set.
     *
     * @return An iterator through all Handles stored in the handle set.
     */
    HandleSetIterator *keys();

    std::string toString();

};


class HandleSetIterator
{

    friend class HandleSet;

private:

    /**
     * Stores the current iterator.
     */
    HandleSet::InternalIterator current;

    /**
     * Stores the handleMap.
     */
    HandleSet *set;

    /**
     * Constructor for this class.
     *
     * @param HandleMap object to be iterated.
     */
    HandleSetIterator(HandleSet *);

public:

    /**
     * Returns whether there still are elements to be iterated.
     *
     * @return Whether there still are elements to be iterated.
     */
    bool hasNext();

    /**
     * Returns the next Handle of the iterator and advances.
     *
     * @return Next Handle of the iterator and advances.
     */
    Handle next() throw (IndexErrorException);
};

} // namespace opencog

#endif // _OPENCOG_HANDLE_SET_H
