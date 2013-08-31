/*
 * opencog/atomspace/HandleMap.h
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

/** \file HandleMap.h
 * red-black tree maps Handle keys to type T elements
 */

#ifndef _OPENCOG_HANDLE_MAP_H
#define _OPENCOG_HANDLE_MAP_H

#include <pthread.h>

#include <map>

#include <opencog/atomspace/types.h>
#include <opencog/util/exceptions.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

template<class T> class HandleMapIterator;

//! red-black tree maps Handle keys to type T elements
/**
 * This is an Adapter to stl's HashMap.
 */
template<class T>
class HandleMap
{
    friend class HandleMapIterator<T>;

private:

    /**
     * Defines an iterator to the map.
     */
    typedef typename std::map< Handle, T, std::less<Handle> > InternalMap;
    typedef typename InternalMap::iterator InternalIterator;

    /**
     * initializes the HandleMap.
     */
    void init(bool use_mutex)
    {
        handle_map = new InternalMap();
        useMutex = use_mutex;
        if (useMutex) pthread_mutex_init(&plock, NULL);
    }

    /**
     * The Map where the elements will be stored.
     */
    InternalMap *handle_map;

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
    HandleMap(bool use_mutex = false)
    {
        init(use_mutex);
    }


    /**
     * Destructor for this class
     */
    ~HandleMap()
    {
        lock();

        delete(handle_map);

        unlock();
    }


    /**
     * Locks the hash map for concurrent access.
     */
    void lock()
    {
        if (useMutex) pthread_mutex_lock(&plock);
    }

    /**
     * Unlocks the hash map for concurrent access.
     */
    void unlock()
    {
        if (useMutex) pthread_mutex_unlock(&plock);
    }


    /**
     * Adds a new entry to the hash table.
     *
     * @param Key.
     * @param Element.
     */
    void add(Handle key, T element) throw (RuntimeException)
    {
        lock();

        // check if the element is already in the hash table. If not, it
        // is added to the head of the list for that position
        if (!contains(key)) {
            (*handle_map)[key] = element;
        } else {
            throw RuntimeException(TRACE_INFO, "attempting to insert duplicated key %d in hash map", key.value());
        }

        unlock();
    }

    /**
     * Returns the element for a given key.
     *
     * @param Key.
     * @return Element for a given key.
     */
    T get(Handle key)
    {
        T ret;

        lock();

        InternalIterator ti = handle_map->find(key);

        // assert the key exists. Otherwise throws an exception.
        if (ti == handle_map->end())
       	{
            unlock();
            throw AssertionException("HandleMap: key (%d) does not exist in this map", key.value());  
        }
        ret = ti->second;

        unlock();
        return ret;
    }


    /**
     * Checks if there exists an element for the given key.
     *
     * @param Key.
     * @return Whether there exists an element for the given key.
     */
    bool contains(Handle key)
    {
        bool ret;

        // lock and call private contains()
        lock();

        InternalIterator ti = handle_map->find(key);

        ret = ti != handle_map->end();

        unlock();

        return ret;
    }

    /**
     * Removes an element referred by a given key from the table and
     * returns it.
     *
     * @param Key.
     * @return Removed element.
     */
    T remove(Handle key)
    {
        T ret = NULL;

        lock();

        InternalIterator ti = handle_map->find(key);

        if (ti != handle_map->end()) {
            ret = ti->second;
            handle_map->erase(ti);
        }

        unlock();

        // returns the removed element.
        return ret;
    }

    /**
     * Changes the size of the hash table to at least a new size.
     *
     * @param New size for the hash table.
     */
    void resize(int newSize)
    {
        lock();

        handle_map->resize(newSize);

        unlock();
    }

    /**
     * Returns the total number of elements in the hash table.
     *
     * @return Total number of elements in the hash table.
     */
    int getCount()
    {
        int size;
        lock();

        size = handle_map->size();

        unlock();
        return(size);
    }

    /**
     * Returns the size of the hash table (number of possible collision
     * lists).
     *
     * @return Size of the hash table (number of possible collision lists).
     */
    int getSize()
    {
        int max_size;
        lock();

        // max_size = handle_map->bucket_count();
        max_size = handle_map->size();

        unlock();

        return max_size;
    }

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
    HandleMapIterator<T> *keys()
    {
        return new HandleMapIterator<T>(this);
    }

};

template<class T>
class HandleMapIterator
{
    friend class HandleMap<T>;

private:

    /**
     * Stores the current iterator.
     */
    typedef typename std::map<Handle, T>::iterator iter_type;
    iter_type current;

    /**
     * Stores the handleMap.
     */
    HandleMap<T> *map;

    /**
     * Constructor for this class.
     *
     * @param HandleMap object to be iterated.
     */
    HandleMapIterator(HandleMap<T> *m)
    {
        map = m;
        current = map->handle_map->begin();
    }

public:

    /**
     * Returns whether there still are elements to be iterated.
     *
     * @return Whether there still are elements to be iterated.
     */
    bool hasNext()
    {
        iter_type e = map->handle_map->end();
        return current != e;
    }


    /**
     * Returns the next key of the iterator and advances.
     *
     * @return Next key of the iterator and advances.
     */
    Handle next() throw (IndexErrorException)
    {
        if (!hasNext()) {
            throw IndexErrorException(TRACE_INFO, "HandleMapIterator out of bounds");
        }

        map->lock();

        Handle ret = current->first;

        ++current;

        map->unlock();

        return ret;
    }

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_HANDLE_MAP_H
