/**
 * HandleMap.h - hashtable based in Handle keys and (void *) elements
 *
 *
 * Copyright (c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#ifndef HANDLEMAP_H
#define HANDLEMAP_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_LIBPTHREAD
#include <pthread.h>
#endif

#include "types.h"
#include "exceptions.h"

class HandleMapIterator;

/**
 * This is an Adapter to stl's HashMap.
 */
class HandleMap {
	friend class HandleMapIterator;
	
private:


	/**
	 * initializes the HandleMap.
	 */
	void init(int initialSize, bool useMutex);

	/**
	 * Defines the default size for HandleMaps.
	 */
    static const int DEFAULT_SIZE = 100;

	/**
	 * Defines the hash_map that will be used.
	 */
    typedef HandleVoidPointerHashMap InternalHashMap;
	
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
	 * Constructor for this class.
	 *
	 * @param Table with at least this size.
	 * @param Whether a concurrent access control mechanism should be
	 * used. Notice that this will introduce a considerable overhead to
	 * all operations in the table.
	 */
	HandleMap(int, bool useMutex = false);
	
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
	void add(Handle, void *) throw (RuntimeException);
	
	/**
	 * Returns the element for a given key.
	 *
	 * @param Key.
	 * @return Element for a given key.
	 */
	void *get(Handle);
	
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
	void *remove(Handle);
	
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
	HandleMapIterator *keys();

};


class HandleMapIterator {
	
    friend class HandleMap;
	
private:

	/**
	 * Stores the current iterator.
	 */
    HandleMap::InternalIterator current;

	/**
	 * Stores the handleMap.
	 */
	HandleMap *map;
	
	/**
	 * Constructor for this class.
	 *
	 * @param HandleMap object to be iterated.
	 */
	HandleMapIterator(HandleMap *);
	
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
