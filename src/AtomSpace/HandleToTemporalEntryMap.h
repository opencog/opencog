/**
 * HandleToTemporalEntryMap.h
 *
 * $Author: Welter
 */
#ifndef HANDLETOTEMPORALENTRYMAP_H
#define HANDLETOTEMPORALENTRYMAP_H

#include "HandleMap.h"
#include "TemporalEntry.h"
#include <string>

class HandleToTemporalEntryMap
{
private :
	HandleMap<TemporalEntry *> *internalMap;
	
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
	HandleMapIterator<TemporalEntry *> * keys();

	/**
	 * Return a copy of the HandleToTemporalEntryMap.
	 */
	HandleToTemporalEntryMap *clone();

	std::string toString();

};


#endif
