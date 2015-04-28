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

#include <map>
#include <memory>
#include <mutex>

#include <opencog/atomspace/Handle.h>
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
    : public std::enable_shared_from_this<HandleMap<T>>
{
    friend class HandleMapIterator<T>;

private:

    /**
     * Defines an iterator to the map.
     */
    typedef typename std::map< Handle, T, std::less<Handle> > InternalMap;
    typedef typename InternalMap::iterator InternalIterator;

    /**
     * The Map where the elements will be stored.
     */
    InternalMap _handle_map;

    /**
     * The mutex used to control access to the HashMap.
     */
    std::mutex _mtx;

public:

    typedef std::shared_ptr<HandleMap<T>> MapPtr;
    HandleMap() {} 

    /**
     * Adds a new entry to the hash table.
     *
     * @param Key.
     * @param Element.
     */
    void add(Handle key, T element) throw (RuntimeException)
    {
        std::lock_guard<std::mutex> lck(_mtx);

        // check if the element is already in the hash table. If not, it
        // is added to the head of the list for that position
        InternalIterator ti = _handle_map.find(key);
        if (ti == _handle_map.end()) {
            (_handle_map)[key] = element;
        } else {
            throw RuntimeException(TRACE_INFO,
                "attempting to insert duplicated key %ld in hash map",
                key.value());
        }
    }

    /**
     * Returns the element for a given key.
     *
     * @param Key.
     * @return Element for a given key.
     */
    T get(Handle key)
    {
        std::lock_guard<std::mutex> lck(_mtx);

        // assert the key exists. Otherwise throws an exception.
        InternalIterator ti = _handle_map.find(key);
        if (ti == _handle_map.end())
        {
            throw AssertionException(
                "HandleMap: key (%ld) does not exist in this map",
                key.value());  
        }
        return ti->second;
    }


    /**
     * Checks if there exists an element for the given key.
     *
     * @param Key.
     * @return Whether there exists an element for the given key.
     */
    bool contains(Handle key)
    {
        std::lock_guard<std::mutex> lck(_mtx);
        return _handle_map.find(key) != _handle_map.end();
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
        std::lock_guard<std::mutex> lck(_mtx);

        T ret = NULL;
        InternalIterator ti = _handle_map.find(key);
        if (ti != _handle_map.end()) {
            ret = ti->second;
            _handle_map.erase(ti);
        }

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
        std::lock_guard<std::mutex> lck(_mtx);
        _handle_map.resize(newSize);
    }

    /**
     * Returns the total number of elements in the hash table.
     *
     * @return Total number of elements in the hash table.
     */
    int getCount()
    {
        std::lock_guard<std::mutex> lck(_mtx);
        return _handle_map.size();
    }

    /**
     * Returns the size of the hash table (number of possible collision
     * lists).  XXX No not any more XXX.
     *
     * @return Size of the hash table (number of possible collision lists).
     */
    int getSize()
    {
        std::lock_guard<std::mutex> lck(_mtx);

        // max_size = handle_map->bucket_count();
        return  _handle_map.size();
    }

    /**
     * Returns an iterator through all keys stored in the hash table.
     *
     * @return An iterator through all keys stored in the hash table.
     */
    HandleMapIterator<T>* keys()
    {
        MapPtr mp = std::enable_shared_from_this<HandleMap<T>>::shared_from_this();
        return new HandleMapIterator<T>(mp);
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
    typedef std::shared_ptr<HandleMap<T>> MapPtr;
    MapPtr map;

    /**
     * Constructor for this class.
     *
     * @param HandleMap object to be iterated.
     */
    HandleMapIterator(MapPtr m)
    {
        map = m;
        std::lock_guard<std::mutex> lck(map->_mtx);
        current = map->_handle_map.begin();
    }

public:

    /**
     * Returns whether there still are elements to be iterated.
     *
     * @return Whether there still are elements to be iterated.
     */
    bool hasNext()
    {
        std::lock_guard<std::mutex> lck(map->_mtx);
        return current != map->_handle_map.end();
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

        std::lock_guard<std::mutex> lck(map->_mtx);

        Handle ret = current->first;
        ++current;
        return ret;
    }

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_HANDLE_MAP_H
