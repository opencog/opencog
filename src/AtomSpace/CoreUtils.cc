/**
 * CoreUtils.cc
 *
 * Module for including any core-specific common utilities
 * 
 * Copyright(c) 2007 Vettalabs
 * All rights reserved.
 */
#include "CoreUtils.h"

void CoreUtils::updateHandle(Handle *handle, HandleMap *handles) throw (RuntimeException) {
    //printf("CoreUtils::updateHandle(%p)\n", *handle);
    if (!handleCompare(handle, &UNDEFINED_HANDLE)) return;

    Handle newH = (Handle)handles->get(*handle);
    if (compare(newH, UNDEFINED_HANDLE)) {
        *handle = newH;
    } else {
        throw RuntimeException(TRACE_INFO, "CoreUtils::updateHandle: unknown handle %p", *handle);
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

