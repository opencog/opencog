/*
 * opencog/spacetime/HandleToTemporalEntryMap.cc
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

#include "HandleToTemporalEntryMap.h"

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomutils/ThreadSafeHandleMap.h>

using namespace opencog;

HandleToTemporalEntryMap::HandleToTemporalEntryMap()
{
    internalMap = std::make_shared<ThreadSafeHandleMap<TemporalEntry*>>();
}

HandleToTemporalEntryMap::~HandleToTemporalEntryMap()
{
    ThreadSafeHandleMapIterator<TemporalEntry*>* keys = internalMap->keys();
    while (keys->has_next()) {
        delete (internalMap->get(keys->next()));
    }
    delete keys;
}

void HandleToTemporalEntryMap::add(Handle key, TemporalEntry* obj)
{
    if (contains(key)) {
        remove(key);
    }
    internalMap->add(key, obj);
}

TemporalEntry* HandleToTemporalEntryMap::get(Handle key)
{
    if (internalMap->contains(key)) 
        return(internalMap->get(key));
    else 
        return(NULL);
}

bool HandleToTemporalEntryMap::contains(Handle key)
{
    return(internalMap->contains(key));
}

TemporalEntry* HandleToTemporalEntryMap::remove(Handle key)
{
    return internalMap->remove(key);
}

int HandleToTemporalEntryMap::getCount()
{
    return(internalMap->get_count());
}

int HandleToTemporalEntryMap::getSize()
{
    return internalMap->get_size();
}

ThreadSafeHandleMapIterator<TemporalEntry *> *HandleToTemporalEntryMap::keys()
{
    return internalMap->keys();
}

HandleToTemporalEntryMap *HandleToTemporalEntryMap::clone()
{
    HandleToTemporalEntryMap *ret = new HandleToTemporalEntryMap();
    ThreadSafeHandleMapIterator<TemporalEntry *> *originalKeys = keys();
    while (originalKeys->has_next()) {
        Handle nextKey = originalKeys->next();
        ret->add(nextKey, get(nextKey));
    }
    delete(originalKeys);
    return(ret);
}

std::string HandleToTemporalEntryMap::toString()
{
    std::string answer;
    for (ThreadSafeHandleMapIterator<TemporalEntry *> *it = keys(); it->has_next();) {
        Handle key = it->next();
        TemporalEntry* value = get(key);
        /* append key */
        AtomPtr atom(key);
        answer += atom->toShortString();
        answer += ":";
        /* append value */
        answer += value->toString();
        if (it->has_next()) {
            answer += ",";
        }
    }
    return answer;
}
