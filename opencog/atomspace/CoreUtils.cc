/*
 * opencog/atomspace/CoreUtils.cc
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

#include "CoreUtils.h"

#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/TLB.h>

using namespace opencog;

/* Module for including any core-specific common utilities */
void CoreUtils::updateHandle(Handle *handle, HandleMap<Atom *> *handles) throw (RuntimeException)
{
    //printf("CoreUtils::updateHandle(%p)\n", *handle);
    //if (TLB::isInvalidHandle(*handle)) return;

    // Assume that the HandleMap stores <Handle, Atom *> pairs ....
    Handle newH = TLB::getHandle(handles->get(*handle));
    if (TLB::isValidHandle(newH)) {
        *handle = newH;
    } else {
        newH = TLB::addAtom(handles->get(*handle));
        if (TLB::isValidHandle(newH)) {
            *handle = newH;
        } else {
            throw RuntimeException(TRACE_INFO, "CoreUtils::updateHandle: unknown handle %p", handle->value());
        }
    }
}

int CoreUtils::handleCompare(const void* e1, const void* e2)
{
    return compare(*((Handle *)e1), *((Handle *)e2));
}

int CoreUtils::compare(Handle h1, Handle h2)
{
    if (h1 < h2) {
        return(-1);
    } else if (h1 > h2) {
        return(1);
    } else {
        return(0);
    }
}

bool CoreUtils::HandleComparison::operator()(const Handle& h1, const Handle& h2) const
{
    return (CoreUtils::compare(h1, h2) < 0);
}

