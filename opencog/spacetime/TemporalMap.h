/*
 * opencog/spacetime/TemporalMap.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Carlos Lopes <dlopes@vettalabs.com>
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

#ifndef _OPENCOG_TEMPORALMAP_H
#define _OPENCOG_TEMPORALMAP_H

#include <pthread.h>

#include <opencog/spacetime/Temporal.h>

#include <boost/unordered_map.hpp>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

class TemporalMapIterator;

/**
 * Hashtable based in Temporal* keys and (void *) elements. 
 */
class TemporalMap
{
    friend class TemporalMapIterator;

private:

    /**
     * initializes the TemporalMap.
     */
    void init(int initialSize, bool useMutex);

    /**
     * Defines the default size for TemporalMaps.
     */
    static const int DEFAULT_SIZE = 100;

    /**
     * Defines the type of the hash map to be used.
     */
    typedef boost::unordered_map<Temporal*, void *, hashTemporal, equalTemporal> InternalHashMap;

public:

    /**
     * Defines an iterator to the hashMap.
     */
    typedef InternalHashMap::iterator InternalIterator;

private:

    /**
     * The hashMap where the elements will be stored.
     */
    InternalHashMap *hashMap;

    /**
     * Indicates whether a mutex for concurrent acces is to be used or not.
     */
    bool useMutex;

    /**
     * The mutex used to control access to the HashMap.
     */
    pthread_mutex_t plock;

public:

    /**
     * Constructor for this class.
     *
     * @param Whether a concurrent access control mechanism should be
     * used. Notice that this will introduce a considerable overhead to
     * all operations in the table.
     */
    TemporalMap(bool useMutex = false);

    /**
     * Constructor for this class.
     *
     * @param Table with at least this size.
     * @param Whether a concurrent access control mechanism should be
     * used. Notice that this will introduce a considerable overhead to
     * all operations in the table.
     */
    TemporalMap(int, bool useMutex = false);

    /**
     * Destructor for this class
     */
    ~TemporalMap();

    /**
     * Locks the hash map for concurrent access.
     */
    void lock();

    /**
     * Unlocks the hash map for concurrent access.
     */
    void unlock();

    /**
     * Adds a new entry to the hash table.
     *
     * @param Key.
     * @param Element.
     */
    void add(Temporal*, void *);

    /**
     * Returns the element for a given key.
     *
     * @param Key.
     * @return Element for a given key.
     */
    void *get(Temporal*);

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
    void *remove(Temporal*);

    /**
     * Changes the size of the hash table to at least a new size.
     *
     * @param New size for the hash table.
     */
    void resize(int);

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
     * Returns whether the hash table is set for concurrent access
     * control.
     *
     * @return Whether the hash table is set for concurrent access
     * control.
     */
    bool getUseMutex();

    /**
     * Returns an iterator through all keys stored in the hash table.
     *
     * @return An iterator through all keys stored in the hash table.
     */
    TemporalMapIterator *keys();

};


class TemporalMapIterator
{

    friend class TemporalMap;

private:

    /**
     * Stores the current iterator.
     */
    TemporalMap::InternalIterator current;

    /**
     * Stores the handleMap.
     */
    TemporalMap *map;

    /**
     * Constructor for this class.
     *
     * @param TemporalMap object to be iterated.
     */
    TemporalMapIterator(TemporalMap *);

public:

    /**
     * Returns whether there still are elements to be iterated.
     *
     * @return Whether there still are elements to be iterated.
     */
    bool hasNext();

    /**
     * Returns the next key of the iterator and advances.
     *
     * @return Next key of the iterator and advances.
     */
    Temporal* next();
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_TEMPORALMAP_H
