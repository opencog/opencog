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
using namespace opencog;

void Trail::init(int initialSize, int max) throw (InvalidParamException, std::bad_exception) {
    if (max < initialSize) {
        throw InvalidParamException(TRACE_INFO, 
                "Trail - maxSize: %d < initialSize: %d.", max, initialSize);
    }
    if (initialSize != 0) {
	    trail = new deque<Handle>(initialSize);
    } else {
    	trail = NULL;
    }
    maxSize = max;
}

Trail::Trail() throw (InvalidParamException, std::bad_exception) {
    init(DEFAULT_INITIAL_TRAIL_SIZE, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize) throw (InvalidParamException, std::bad_exception) {
    init(initialSize, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize, int max) throw (InvalidParamException, std::bad_exception) {
    init(initialSize, max);
}

Trail::~Trail() {
     if (trail != NULL) delete trail;
}

bool Trail::isInTrail(Handle link){
	
	if (trail == NULL) return false;
	   
    for (int i = 0; i < (int)trail->size(); i++) {
        if ((TLB::getAtom(link)->equals(TLB::getAtom((*trail)[i])))){
			return(true);
        } 
    }
	
    return false;
}

int Trail::getSize() {
    
    if (trail == NULL) return 0;
    
    return (int)trail->size();
}

void Trail::insert(Handle link, bool fresh){
	if (trail == NULL) init(1, maxSize);
	
	if (fresh){
		if(isInTrail(link)) return; 
	}
	
	trail->push_back(link);
    
    if (maxSize < (int) trail->size()) {
        trail->pop_front();
    }
    
}

Handle Trail::getElement(int index) throw (IndexErrorException, std::bad_exception) {

	if (trail == NULL) {
        throw IndexErrorException(TRACE_INFO, 
                "Trail - invalid out of bounds (%d > %d).",
                index, 0);
    }
	    
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

	if (trail == NULL) init(1, maxSize);

    for(int i = 0; i < t->getSize(); i++) {
        Handle link = t->getElement(i);
        insert(link);
    }
}

void Trail::print() {

	if (trail == NULL) {
    	logger().debug("trailSize %d\n", 0);

	} else {
    	logger().debug("trailSize %d\n", (int)trail->size());
	    for(int i = 0; i < (int)trail->size(); i++) {
			logger().info("%d. %s\n", i, ((Link*)((*trail)[i]))->toShortString().c_str());
	    }
	}

    logger().info("endOfTrail\n\n");
}


void Trail::print(FILE* fp) {

	if (trail == NULL) return;

    for(int i = 0; i < (int)trail->size(); i++) {
        fprintf(fp, "\t%d. %s\n", i, ((Link*)((*trail)[i]))->toShortString().c_str());
    }
}
