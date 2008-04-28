/**
 * CoreUtils.cc
 *
 * Module for including any core-specific common utilities
 * 
 * Copyright(c) 2007 Vettalabs
 * All rights reserved.
 */
#include "CoreUtils.h"
#include "TLB.h"
#include "HandleMap.cc"

void CoreUtils::updateHandle(Handle *handle, HandleMap<Atom *> *handles) throw (RuntimeException)
{
    //printf("CoreUtils::updateHandle(%p)\n", *handle);
    if (TLB::isInvalidHandle(*handle)) return;

    // Assume that the HandleMap stores <Handle, Atom *> pairs ....
    Handle newH = TLB::getHandle(handles->get(*handle));
    if (TLB::isValidHandle(newH)) {
        *handle = newH;
    } else {
        newH = TLB::addAtom(handles->get(*handle));
        if (TLB::isValidHandle(newH)) {
            *handle = newH;
        } else {
            throw RuntimeException(TRACE_INFO, "CoreUtils::updateHandle: unknown handle %p", *handle);
        }
    }
}

int CoreUtils::handleCompare(const void* e1, const void* e2) {
    return compare(*((Handle *)e1), *((Handle *)e2));
}

int CoreUtils::compare(Handle h1, Handle h2) {
    if (h1 < h2){
        return(-1);
    }else if (h1 > h2){
        return(1);
    }else{
        return(0);
    }
}

bool CoreUtils::HandleComparison::operator()(const Handle& h1, const Handle& h2) const {
    return(CoreUtils::compare(h1, h2) > 0);
}

