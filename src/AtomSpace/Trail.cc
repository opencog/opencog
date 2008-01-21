/**
* Trail.cc
*
*
* Copyright(c) 2001 Guilherme Lamacie
* All rights reserved.
*/

#include "Trail.h"
#include "Atom.h"
#include "Link.h"
#include <deque>
#include <stdio.h>
#include "types.h"
#include "TLB.h"
#include "ClassServer.h"
#include "Logger.h"

#define DEFAULT_INITIAL_TRAIL_SIZE 0
#define DEFAULT_MAX_TRAIL_SIZE 1000

using namespace std;

void Trail::init(int initialSize, int max) throw (InvalidParamException) {
    if (max < initialSize) {
        throw InvalidParamException(TRACE_INFO, 
                "Trail - maxSize: %d < initialSize: %d.", max, initialSize);
    }
    trail = new deque<Handle>(initialSize);
    maxSize = max;
}

Trail::Trail() throw (InvalidParamException) {
    init(DEFAULT_INITIAL_TRAIL_SIZE, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize) throw (InvalidParamException) {
    init(initialSize, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize, int max) throw (InvalidParamException) {
    init(initialSize, max);
}

Trail::~Trail() {
     delete trail;
}

bool Trail::isInTrail(Handle link){
   
    for (int i = 0; i < (int)trail->size(); i++) {
        if ((TLB::getAtom(link)->equals(TLB::getAtom((*trail)[i])))){
			return(true);
        } 
    }
	
    return false;
}

int Trail::getSize() {
    return (int)trail->size();
}

void Trail::insert(Handle link, bool fresh){
	if (fresh){
		if(isInTrail(link)) return; 
	}
	
	trail->push_back(link);
    
    if (maxSize < (int) trail->size()) {
        trail->pop_front();
    }
    
}

Handle Trail::getElement(int index) throw (IndexErrorException) {
    
    if(index > (int)trail->size()) {
        throw IndexErrorException(TRACE_INFO, 
                "Trail - invalid out of bounds (%d > %d).",
                index, trail->size());
    }
    return (*trail)[index];
}

void Trail::append(Trail* t) {
    if(!t->getSize()) {
        return; 
    }
    for(int i = 0; i < t->getSize(); i++) {
        Handle link = t->getElement(i);
        insert(link);
    }
}

void Trail::print() {
    MAIN_LOGGER.log(Util::Logger::DEBUG, "trailSize %d\n", (int)trail->size());
    for(int i = 0; i < (int)trail->size(); i++) {
		MAIN_LOGGER.log(Util::Logger::INFO, "%d. %s\n", i, ((Link*)((*trail)[i]))->toShortString().c_str());
    }
    MAIN_LOGGER.log(Util::Logger::INFO, "endOfTrail\n\n");
}


void Trail::print(FILE* fp) {

    for(int i = 0; i < (int)trail->size(); i++) {
        fprintf(fp, "\t%d. %s\n", i, ((Link*)((*trail)[i]))->toShortString().c_str());
    }
}
