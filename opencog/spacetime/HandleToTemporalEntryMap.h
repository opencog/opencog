/*
 * opencog/spacetime/HandleToTemporalEntryMap.h
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

#ifndef _OPENCOG_HANDLE_TO_TEMPORAL_ENTRY_MAP_H
#define _OPENCOG_HANDLE_TO_TEMPORAL_ENTRY_MAP_H

#include <string>

#include <opencog/atomutils/ThreadSafeHandleMap.h>
#include <opencog/spacetime/TemporalEntry.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

class HandleToTemporalEntryMap
{

private:

    ThreadSafeHandleMap<TemporalEntry*>::MapPtr internalMap;

public:

    /**
     * Constructor for this class.
     */
    HandleToTemporalEntryMap();

    ~HandleToTemporalEntryMap();

    /**
     * @param Key.
     * @param Element.
     */
    void add(Handle, TemporalEntry*);

    /**
     * Returns the element for a given key.
     *
     * @param Key.
     * @return Element for a given key.
     */
    TemporalEntry* get(Handle);

    /**
     * Checks if there exists an element for the given key.
     *
     * @param Key.
     * @return Whether there exists an element for the given key.
     */
    bool contains(Handle);

    /**
     * Removes an element referred by a given key from the table and
     * returns it.
     *
     * @param Key.
     * @return Removed element.
     */
    TemporalEntry* remove(Handle);

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
    ThreadSafeHandleMapIterator<TemporalEntry *> * keys();

    /**
     * Return a copy of the HandleToTemporalEntryMap.
     */
    HandleToTemporalEntryMap *clone();

    std::string toString();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_HANDLE_TO_TEMPORAL_ENTRY_MAP_H
