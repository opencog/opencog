/**
 * HandleToTemporalEntryMap.cc
 *
 * $Author: Welter
 */
#include <HandleToTemporalEntryMap.h>
#include <TLB.h>
#include "HandleMap.cc"

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
    if (contains(key))
    {
        remove(key);
    }
    internalMap->add(key, obj);
}
    
TemporalEntry* HandleToTemporalEntryMap::get(Handle key)
{
    return(internalMap->get(key));
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
    while (originalKeys->hasNext())
    {
        Handle nextKey = originalKeys->next();
        ret->add(nextKey, get(nextKey));
    }
    delete(originalKeys);
    return(ret);
}

std::string HandleToTemporalEntryMap::toString()
{
    std::string answer;
    for (HandleMapIterator<TemporalEntry *> *it = keys(); it->hasNext();)
    {
        Handle key = it->next();
        TemporalEntry* value = get(key);
        /* append key */
        Atom* atom = TLB::getAtom(key);
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
