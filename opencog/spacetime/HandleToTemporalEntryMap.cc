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

#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/TLB.h>

using namespace opencog;

HandleToTemporalEntryMap::HandleToTemporalEntryMap()
{
    internalMap = new HandleMap<TemporalEntry *>();
}

HandleToTemporalEntryMap::~HandleToTemporalEntryMap()
{
    HandleMapIterator<TemporalEntry *>* keys = internalMap->keys();
    while (keys->hasNext()) {
        delete (internalMap->get(keys->next()));
    }
    delete keys;
    delete(internalMap);
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
    return(internalMap->getCount());
}

int HandleToTemporalEntryMap::getSize()
{
    return internalMap->getSize();
}

HandleMapIterator<TemporalEntry *> *HandleToTemporalEntryMap::keys()
{
    return internalMap->keys();
}

HandleToTemporalEntryMap *HandleToTemporalEntryMap::clone()
{
    HandleToTemporalEntryMap *ret = new HandleToTemporalEntryMap();
    HandleMapIterator<TemporalEntry *> *originalKeys = keys();
    while (originalKeys->hasNext()) {
        Handle nextKey = originalKeys->next();
        ret->add(nextKey, get(nextKey));
    }
    delete(originalKeys);
    return(ret);
}

std::string HandleToTemporalEntryMap::toString()
{
    std::string answer;
    for (HandleMapIterator<TemporalEntry *> *it = keys(); it->hasNext();) {
        Handle key = it->next();
        TemporalEntry* value = get(key);
        /* append key */
        AtomPtr atom = TLB::getAtom(key);
        answer += atom->toShortString();
        answer += ":";
        /* append value */
        answer += value->toString();
        if (it->hasNext()) {
            answer += ",";
        }
    }
    return answer;
}
