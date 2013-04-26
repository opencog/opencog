/*
 * opencog/spacetime/TemporalToHandleSetMap.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _OPENCOG_TEMPORAL_TO_HANDLE_SET_MAP_H
#define _OPENCOG_TEMPORAL_TO_HANDLE_SET_MAP_H

#include <string>

#include <opencog/atomspace/Handle.h>
#include <opencog/spacetime/TemporalMap.h>

namespace opencog
{

class TemporalToHandleSetMap
{

private:

    TemporalMap* internalMap;

public:

    /**
     * Constructor for this class.
     */
    TemporalToHandleSetMap();

    ~TemporalToHandleSetMap();

    /**
     * @param Key.
     * @param Element.
     */
    void add(Temporal*, UnorderedHandleSet*);

    /**
     * Returns the element for a given key.
     *
     * @param Key.
     * @return Element for a given key.
     */
    UnorderedHandleSet* get(Temporal*);

    /**
     * Returns the key Temporal object equals to the given parameter.
     *
     * @param t    reference to a Temporal object to make the lookup.
     * @return pointer to the searched key Temporal object, if it exists. NULL, otherwise.
     */
    Temporal *getKey(const Temporal&);

    /**
     * Checks if there exists an element for the given key.
     *
     * @param Key.
     * @return Whether there exists an element for the given key.
     */
    bool contains(Temporal*);

    /**
     * Removes an element referred by a given key from the table and
     * returns it.
     *
     * @param Key.
     * @return Removed element.
     */
    UnorderedHandleSet* remove(Temporal*);

    /**
     * Returns the total number of elements in the hash table.
     *
     * @return Total number of elements in the hash table.
     */
    int getCount();

    /**
     * Returns the size of the hash table (number of possible collision
     * lists).
     *
     * @return Size of the hash table (number of possible collision lists).
     */
    int getSize();


    /**
     * Returns an iterator through all keys stored in the hash table.
     *
     * @return An iterator through all keys stored in the hash table.
     */
    TemporalMapIterator *keys();

    /**
     * Return a copy of the TemporalToHandleSetMap.
     */
    TemporalToHandleSetMap *clone();

    std::string toString();

};

} // namespace opencog

#endif // _OPENCOG_TEMPORAL_TO_HANDLE_SET_MAP_H
