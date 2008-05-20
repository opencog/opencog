/*
 * src/AtomSpace/HandleMap.h
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

/** HandleMap.h - red-black tree maps Handle keys to type T elements */

#ifndef HANDLEMAP_H
#define HANDLEMAP_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_LIBPTHREAD
#include <pthread.h>
#endif

#include <map>

#include "types.h"
#include "exceptions.h"

template<class T>
class HandleMapIterator;

	
/**
 * This is an Adapter to stl's HashMap.
 */
template<class T>
class HandleMap
{
	friend class HandleMapIterator<T>;
	
private:


	/**
	 * initializes the HandleMap.
	 */
	void init(bool useMutex);

public:
	
	/**
	 * Defines an iterator to the map.
	 */
   typedef std::map<Handle, T> InternalMap;
	typedef typename InternalMap::iterator InternalIterator;
	
	
private:
	
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
#ifdef HAVE_LIBPTHREAD
	pthread_mutex_t plock;
#endif
	
public:
	
	/**
	 * Constructor for this class.
	 *
	 * @param Whether a concurrent access control mechanism should be
	 * used. Notice that this will introduce a considerable overhead to
	 * all operations in the table.
	 */
	HandleMap(bool useMutex = false);
	
	/**
	 * Destructor for this class
	 */
	~HandleMap();
	
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
	void add(Handle, T) throw (RuntimeException);
	
	/**
	 * Returns the element for a given key.
	 *
	 * @param Key.
	 * @return Element for a given key.
	 */
	T get(Handle);
	
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
	T remove(Handle);
	
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
	HandleMapIterator<T> *keys();

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
	HandleMapIterator(HandleMap<T> *);
	
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
	Handle next() throw (IndexErrorException);
};

#endif
