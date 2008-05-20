/*
 * src/AtomSpace/types.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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
