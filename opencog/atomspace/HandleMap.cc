/*
 * opencog/atomspace/HandleMap.cc
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

#include "HandleMap.h"

#include <stdio.h>
#include <stdlib.h>

/** HandleMap.cc - hashtable based in integer keys and (void *) elements */

using namespace opencog;

template <class T>
void HandleMap<T>::lock()
{
#ifdef HAVE_LIBPTHREAD
    if (useMutex) {
        pthread_mutex_lock(&plock);
    }
#endif
}

template <class T>
void HandleMap<T>::unlock()
{
#ifdef HAVE_LIBPTHREAD
    if (useMutex) {
        pthread_mutex_unlock(&plock);
    }
#endif
}


template <class T>
void HandleMap<T>::init(bool use_mutex)
{
    handle_map = new InternalMap();

    useMutex = use_mutex;

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&plock, NULL);
#endif
}

template <class T>
HandleMap<T>::~HandleMap()
{

    lock();

    delete(handle_map);

    unlock();
}

template <class T>
HandleMap<T>::HandleMap(bool use_mutex)
{
    init(use_mutex);
}

template <class T>
void HandleMap<T>::add(Handle key, T element) throw (RuntimeException)
{
    lock();

    // check if the element is already in the hash table. If not, it
    // is added to the head of the list for that position
    if (!contains(key)) {
        (*handle_map)[key] = element;
    } else {
        throw RuntimeException(TRACE_INFO, "attempting to insert duplicated key %d in hash map", key);
    }

    unlock();
}

template <class T>
T HandleMap<T>::get(Handle key)
{
    T ret;

    lock();

    InternalIterator ti = handle_map->find(key);

    // if the key is not found, return NULL.
    if (ti == handle_map->end())
        return ((T) NULL); // double cast doesn't work in 64bit
    ret = ti->second;

    unlock();
    return ret;
}

template <class T>
bool HandleMap<T>::contains(Handle key)
{
    bool ret;

    // lock and call private contains()
    lock();

    InternalIterator ti = handle_map->find(key);

    ret = ti != handle_map->end();

    unlock();

    return ret;
}

template <class T>
T HandleMap<T>::remove(Handle key)
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

template <class T>
void HandleMap<T>::resize(int newSize)
{
    lock();

    handle_map->resize(newSize);

    unlock();
}

template <class T>
int HandleMap<T>::getCount()
{
    int size;
    lock();

    size = handle_map->size();

    unlock();
    return(size);
}

template <class T>
int HandleMap<T>::getSize()
{
    int max_size;
    lock();

    // max_size = handle_map->bucket_count();
    max_size = handle_map->size();

    unlock();

    return max_size;
}

template <class T>
HandleMapIterator<T> *HandleMap<T>::keys()
{
    return new HandleMapIterator<T>(this);
}

template <class T>
HandleMapIterator<T>::HandleMapIterator(HandleMap<T> *m)
{
    map = m;
    current = map->handle_map->begin();
}

template <class T>
bool HandleMapIterator<T>::hasNext()
{
    iter_type e = map->handle_map->end();
    return current != e;
}

template <class T>
Handle HandleMapIterator<T>::next() throw (IndexErrorException)
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

