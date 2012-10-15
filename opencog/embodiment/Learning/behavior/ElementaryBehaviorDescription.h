/*
 * opencog/embodiment/Learning/behavior/ElementaryBehaviorDescription.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef ELEMENTARYBEHAVIORDESCRIPTION_H
#define ELEMENTARYBEHAVIORDESCRIPTION_H

#include <opencog/util/StringManipulator.h>

#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/Temporal.h>

using namespace opencog;

namespace behavior
{

class ElementaryBehaviorDescription
{

public:

    Handle handle;
    Temporal temporal;

    ElementaryBehaviorDescription() : temporal(0) {}
    ElementaryBehaviorDescription(Handle h, const Temporal& t): handle(h), temporal(t) {}

    std::string toString(AtomSpace& a) const {
        std::string str = std::string("{") +
                          (handle == Handle::UNDEFINED ? std::string("Handle::UNDEFINED") :
                          a.atomAsString(handle))
                          + std::string(",") + temporal.toString() + std::string("}");
        return str;
    }

}; // class
}  // namespace

#endif
