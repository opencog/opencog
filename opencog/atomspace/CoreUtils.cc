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
// XXX this should be deprecated/made obsolete ... Why?
// Because its a bizarre hack having to do with how the
// load-from-file code works. The load-from-file stuff
// should go into its own directory. Why?  Because it is doing 
// soemthing fairly "illegal" -- monkeying with Handle assignments
// in really dangerous ways. -- and worse, in dangerous, 
// hard-to-understand ways.  This means that whenever something
// about TLB or Handles changes, this code is in the path of 
// breaken stuff, and thus has to be repeatedly debugged and fixed. 
// This sucks. Its a bad design.  The save-to-file, load-from-file
// code needs to be redesigned, and this re-jiggering should go. 
// At least, go away from here. XXXX
void CoreUtils::updateHandle(Handle *handle, HandleMap<Atom *> *handles) throw (RuntimeException)
{
    //printf("CoreUtils::updateHandle(%p)\n", *handle);
    //if (TLB::isInvalidHandle(*handle)) return;

    Atom *a  = handles->get(*handle);
    // Assume that the HandleMap stores <Handle, Atom *> pairs ....
    Handle newH = TLB::getHandle(a);
    if (TLB::isValidHandle(newH)) {
        *handle = newH;
    } else {
        newH = TLB::addAtom(a);
        if (TLB::isValidHandle(newH)) {
            *handle = newH;
        } else {
            throw RuntimeException(TRACE_INFO, "CoreUtils::updateHandle: unknown handle %p", handle->value());
        }
    }
}

