/**
 * HandleSet.cc
 *
 *
 * Author: Rodrigo Barra
 * Copyright(c) 2004 Vettatech Technologies
 * All rights reserved.
 */
#include <HandleSet.h>

HandleSet::HandleSet(){
	handleSet = new InternalHandleSet();
}

HandleSet::HandleSet(InternalHandleSet *set){
	handleSet = set;
}

HandleSet::~HandleSet(){
	delete(handleSet);
}

HandleSet *HandleSet::clone(){
	return(new HandleSet(new InternalHandleSet(*handleSet)));
}

void HandleSet::add(Handle h){
	handleSet->insert(h);
}

void HandleSet::add(HandleSet *hs){
	handleSet->insert(hs->handleSet->begin(), hs->handleSet->end());
}

bool HandleSet::contains(Handle h) const{
	InternalIterator it= handleSet->find(h);
	
	return(it != handleSet->end());
}


void HandleSet::remove(Handle h) throw (RuntimeException){
	InternalIterator it= handleSet->find(h);
	
	if (it != handleSet->end()){
		handleSet->erase(it);
	}else{
		throw RuntimeException(TRACE_INFO, "Could not remove inexistent Handle");
	}
}
	
int HandleSet::getSize(){
	return(handleSet->size());
}

HandleSetIterator *HandleSet::keys(){
	return new HandleSetIterator(this);
}

std::string HandleSet::toString() {
    std::string answer;
    for (HandleSetIterator *it = keys(); it->hasNext();) {
        Handle key = it->next();
        /* append key */
        answer += key->toShortString();
        if (it->hasNext()) {
            answer += ",";
        }
    }
    return answer;
}


HandleSetIterator::HandleSetIterator(HandleSet *set){
	this->set = set;
	current = set->handleSet->begin();
}

bool HandleSetIterator::hasNext(){
	return current != set->handleSet->end();
}
	
Handle HandleSetIterator::next() throw (IndexErrorException){
	if (!hasNext()) {
        throw IndexErrorException(TRACE_INFO, "HandleSetIterator out of bounds");
    } 	

	Handle ret = *current;
	
	current++;

	return(ret);
}
