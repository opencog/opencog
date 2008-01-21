/**
 * HandleSet.h
 *
 *
 * Author: Rodrigo Barra
 * Copyright(c) 2004 Vettatech Technologies
 * All rights reserved.
 */
#ifndef HANDLESET_H
#define HANDLESET_H

#include <utils.h>
#include <TLB.h>
#include <types.h>

#ifndef WIN32
#include <ext/hash_set>
using __gnu_cxx::hash_set;
#endif

class HandleSetIterator;

class HandleSet {
	friend class HandleSetIterator;

private:
	/**
	 * Defines a hash set used to store handles.
	 */
	typedef hash_set<Handle, hashHandle, eqHandle> InternalHandleSet;

	
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


class HandleSetIterator {
	
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
	 * Returns the next Hasndle of the iterator and advances.
	 *
	 * @return Next Handle of the iterator and advances.
	 */
	Handle next() throw (IndexErrorException); 
};

#endif //HANDLESET_H
