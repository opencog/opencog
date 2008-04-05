/**
 * types.cc
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */
#include "types.h"

int hashHandle::operator()(Handle h) const{
    int hashCode =  Util::hash<unsigned long>()((unsigned long) h);
    return(hashCode);
}

bool eqHandle::operator()(Handle h1, Handle h2) const{
    return (h1 == h2);
}

float ShortFloatOps::getValue(const ShortFloat *x) {
	return(*x);
}

void ShortFloatOps::setValue(ShortFloat *x, float value) {
    *x = value; 
}
