/**
 * HandleMap.cc - hashtable based in integer keys and (void *) elements
 *
 *
 * Copyright (c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include "HandleMap.h"

inline void HandleMap::lock() {
#ifdef HAVE_LIBPTHREAD
	if (useMutex){
		pthread_mutex_lock(&plock);
	}
#endif
}

inline void HandleMap::unlock() {
#ifdef HAVE_LIBPTHREAD
	if (useMutex){
		pthread_mutex_unlock(&plock);
    }
#endif
}


void HandleMap::init(int initialSize, bool useMutex) {
	
	hashMap = new InternalHashMap(initialSize);
	
	this->useMutex = useMutex;
	
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&plock, NULL);
#endif
}

HandleMap::~HandleMap() {
	
    lock();
	
	delete(hashMap);
	
    unlock();
}

HandleMap::HandleMap(bool useMutex) {
    init(DEFAULT_SIZE, useMutex);
}

HandleMap::HandleMap(int size, bool useMutex) {
    init(size, useMutex);
}

void HandleMap::add(Handle key, void *element) throw (RuntimeException){
	
    lock();
	
    // check if the element is already in the hash table. If not, it
    // is added to the head of the list for that position
    if (!contains(key)) {
		(*hashMap)[key] = element;
    } else {
        throw RuntimeException(TRACE_INFO, "attempting to insert duplicated key %d in hash map", key);
    }
	
    unlock();
}

void *HandleMap::get(Handle key) {
	
	void *ret;
	
    lock();
	
	InternalIterator ti = hashMap->find(key);
	
	if (ti == hashMap->end()){
		return(NULL);
	}
	
	ret = ti->second;
	
    unlock();
	
    // if the key is not found, return NULL.
    return ret;
}

bool HandleMap::contains(Handle key) {
	
	bool ret;
	
    // lock and call private contains()
    lock();
	
	InternalIterator ti = hashMap->find(key);
	
	ret = ti != hashMap->end();
	
	unlock();

    return ret;
}

void *HandleMap::remove(Handle key) {
	void *ret = NULL;
	
    lock();
	
	InternalIterator ti = hashMap->find(key);
	
	if (ti != hashMap->end()){
		ret = ti->second;
		hashMap->erase(ti);
	}
	
    unlock();
	
    // returns the removed element.
    return ret;
}

void HandleMap::resize(int newSize) {
    lock();

	hashMap->resize(newSize);
	
    unlock();
}

int HandleMap::getCount() {
    int size;
	lock();

    size = hashMap->size();

	unlock();
	return(size);
}

int HandleMap::getSize() {
	int max_size;
	lock();
	
	max_size = hashMap->bucket_count();
	
	unlock();
	
	return max_size;
}


HandleMapIterator *HandleMap::keys() {
	return new HandleMapIterator(this);
}

HandleMapIterator::HandleMapIterator(HandleMap *m){
	map = m;
	current = map->hashMap->begin();
}

bool HandleMapIterator::hasNext() {
    return current != map->hashMap->end();
}

Handle HandleMapIterator::next() throw (IndexErrorException){
	
    if (!hasNext()) {
        throw IndexErrorException(TRACE_INFO, "HandleMapIterator out of bounds");
    } 
	
    map->lock();

    Handle ret = current->first;

	current++;
    
	map->unlock();

    return ret;
}

