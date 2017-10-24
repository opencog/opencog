/*
 * opencog/spacetime/TemporalMap.cc
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

#include <stdio.h>
#include <stdlib.h>

#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>

#include "TemporalMap.h"

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

inline void TemporalMap::lock()
{
    if (useMutex) {
        pthread_mutex_lock(&plock);
    }
}

inline void TemporalMap::unlock()
{
    if (useMutex) {
        pthread_mutex_unlock(&plock);
    }
}


void TemporalMap::init(int initialSize, bool useMutex)
{

    hashMap = new InternalHashMap(initialSize);

    this->useMutex = useMutex;

    pthread_mutex_init(&plock, NULL);
}

TemporalMap::~TemporalMap()
{

    lock();

    delete(hashMap);

    unlock();
}

TemporalMap::TemporalMap(bool useMutex)
{
    init(DEFAULT_SIZE, useMutex);
}

TemporalMap::TemporalMap(int size, bool useMutex)
{
    init(size, useMutex);
}

void TemporalMap::add(Temporal* key, void *element)
{

    lock();

    // check if the element is already in the hash table. If not, it
    // is added to the head of the list for that position
    if (!contains(key)) {
        (*hashMap)[key] = element;
    } else {
        throw RuntimeException(TRACE_INFO,
                               "TemporalMap - Attempting to insert duplicated key in hash map: '%d'.", key);
    }

    unlock();
}

void *TemporalMap::get(Temporal* key)
{
    DPRINTF("TemporalMap::get(%s)\n", key->toString().c_str());

    void *ret;

    lock();

    InternalIterator ti = hashMap->find(key);

    if (ti == hashMap->end()) {
        // if the key is not found, return NULL.
        ret = NULL;
    } else {
        ret = ti->second;
    }

    unlock();

    return ret;
}

Temporal *TemporalMap::getKey(const Temporal& lookupKey)
{
    DPRINTF("TemporalMap::getKey(%s)\n", lookupKey.toString().c_str());

    Temporal *ret;

    lock();

    InternalIterator ti = hashMap->find((Temporal*) & lookupKey);

    if (ti == hashMap->end()) {
        // if the key is not found, return NULL.
        ret = NULL;
    } else {
        ret = ti->first;
    }

    unlock();

    return ret;
}

bool TemporalMap::contains(Temporal* key)
{

    bool ret;

    // lock and call private contains()
    lock();

    InternalIterator ti = hashMap->find(key);

    ret = ti != hashMap->end();

    unlock();

    return ret;
}

void *TemporalMap::remove(Temporal* key)
{
    void *ret = NULL;

    lock();

    InternalIterator ti = hashMap->find(key);

    if (ti != hashMap->end()) {
        ret = ti->second;
        hashMap->erase(ti);
    }

    unlock();

    // returns the removed element.
    return ret;
}

int TemporalMap::getCount()
{
    int size;
    lock();

    size = hashMap->size();

    unlock();
    return(size);
}

int TemporalMap::getSize()
{
    int max_size;
    lock();

    max_size = hashMap->bucket_count();

    unlock();

    return max_size;
}


TemporalMapIterator *TemporalMap::keys()
{
    return new TemporalMapIterator(this);
}

TemporalMapIterator::TemporalMapIterator(TemporalMap *m)
{
    map = m;
    current = map->hashMap->begin();
}

bool TemporalMapIterator::hasNext()
{
    return current != map->hashMap->end();
}

Temporal* TemporalMapIterator::next()
{

    if (!hasNext()) {
        throw IndexErrorException(TRACE_INFO, "TemporalMap - Iterator out of bounds.");
    }

    map->lock();

    Temporal* ret = current->first;

    ++current;

    map->unlock();

    return ret;
}
