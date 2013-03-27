/*
 * opencog/spacetime/TemporalToHandleSetMap.cc
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

#include <sstream>
#include "TemporalToHandleSetMap.h"

using namespace opencog;

TemporalToHandleSetMap::TemporalToHandleSetMap()
{
    internalMap = new TemporalMap();
}

TemporalToHandleSetMap::~TemporalToHandleSetMap()
{
    TemporalMapIterator* keys = internalMap->keys();
    while (keys->hasNext()) {
        delete ((UnorderedHandleSet*) internalMap->get(keys->next()));
    }
    delete(keys);
    delete(internalMap);
}

void TemporalToHandleSetMap::add(Temporal* key, UnorderedHandleSet* obj)
{
    if (contains(key)) {
        remove(key);
    }
    internalMap->add(key, (void *) obj);
}

UnorderedHandleSet* TemporalToHandleSetMap::get(Temporal* key)
{
    return((UnorderedHandleSet*) internalMap->get(key));
}

Temporal* TemporalToHandleSetMap::getKey(const Temporal& lookupKey)
{
    return internalMap->getKey(lookupKey);
}

bool TemporalToHandleSetMap::contains(Temporal* key)
{
    return(internalMap->contains(key));
}

UnorderedHandleSet* TemporalToHandleSetMap::remove(Temporal* key)
{
    return((UnorderedHandleSet*) internalMap->remove(key));
}

int TemporalToHandleSetMap::getCount()
{
    return(internalMap->getCount());
}

int TemporalToHandleSetMap::getSize()
{
    return(internalMap->getSize());
}

TemporalMapIterator *TemporalToHandleSetMap::keys()
{
    return(internalMap->keys());
}

TemporalToHandleSetMap *TemporalToHandleSetMap::clone()
{
    TemporalToHandleSetMap *ret = new TemporalToHandleSetMap();
    TemporalMapIterator *originalKeys = keys();
    while (originalKeys->hasNext()) {
        Temporal* nextKey = originalKeys->next();
        ret->add(nextKey, get(nextKey));
    }
    delete(originalKeys);
    return(ret);
}

std::string TemporalToHandleSetMap::toString()
{
    std::string answer;
    for (TemporalMapIterator *it = keys(); it->hasNext();) {
        Temporal* key = it->next();
        UnorderedHandleSet* value = get(key);
        /* append key */
        answer += key->toString();
        answer += ":";
        /* append value */
        std::stringstream ss;
        ss << value;
        answer += ss.str();
        if (it->hasNext()) {
            answer += ",";
        }
    }
    return answer;
}
