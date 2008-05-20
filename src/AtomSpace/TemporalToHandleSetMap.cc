/**
 * TemporalToHandleSetMap.cc
 *
 * $Author: Welter
 */
#include <TemporalToHandleSetMap.h>

TemporalToHandleSetMap::TemporalToHandleSetMap(){
    internalMap = new TemporalMap();
}

TemporalToHandleSetMap::~TemporalToHandleSetMap(){
    TemporalMapIterator* keys = internalMap->keys();
    while (keys->hasNext()) {
        delete ((HandleSet*) internalMap->get(keys->next())); 
    }
    delete(keys);
    delete(internalMap);
}
    
void TemporalToHandleSetMap::add(Temporal* key, HandleSet* obj){
    if (contains(key)){
        remove(key);
    }
    internalMap->add(key, (void *) obj);
}
    
HandleSet* TemporalToHandleSetMap::get(Temporal* key){
    return((HandleSet*) internalMap->get(key));
}
    
Temporal* TemporalToHandleSetMap::getKey(const Temporal& lookupKey){
    return internalMap->getKey(lookupKey);
}
    
bool TemporalToHandleSetMap::contains(Temporal* key){
    return(internalMap->contains(key));
}
    
HandleSet* TemporalToHandleSetMap::remove(Temporal* key){
    return((HandleSet*) internalMap->remove(key));
}
    
int TemporalToHandleSetMap::getCount(){
    return(internalMap->getCount());
}
    
int TemporalToHandleSetMap::getSize(){
    return(internalMap->getSize());
}

TemporalMapIterator *TemporalToHandleSetMap::keys(){
    return(internalMap->keys());
}

TemporalToHandleSetMap *TemporalToHandleSetMap::clone(){
    TemporalToHandleSetMap *ret = new TemporalToHandleSetMap();
    TemporalMapIterator *originalKeys = keys();
    while (originalKeys->hasNext()){
        Temporal* nextKey = originalKeys->next();
        ret->add(nextKey, get(nextKey));
    }
    delete(originalKeys);
    return(ret);
}

std::string TemporalToHandleSetMap::toString() {
    std::string answer;
    for (TemporalMapIterator *it = keys(); it->hasNext();) {
        Temporal* key = it->next();
        HandleSet* value = get(key);
        /* append key */
        answer += key->toString();
        answer += ":";
        /* append value */
        answer += value->toString();
        if (it->hasNext()) {
            answer += ",";
        }
    }
    return answer;
}
