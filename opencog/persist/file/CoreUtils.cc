/*
 * opencog/perist/file/CoreUtils.cc
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
void CoreUtils::updateHandle(Handle* handle, HandMapPtr handles) throw (RuntimeException)
{
    AtomPtr a(handles->get(*handle));
    // Assume that the HandleMap stores <Handle, Atom *> pairs ....
    Handle newH = a->getHandle();
    if (Handle::UNDEFINED != newH) {
        *handle = newH;
    } else {
        newH = a->getHandle();
        if (Handle::UNDEFINED != newH) {
            *handle = newH;
        } else {
            throw RuntimeException(TRACE_INFO, "CoreUtils::updateHandle: "
                    "unknown handle %p", handle->value());
        }
    }
}

