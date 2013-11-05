/*
 * opencog/atomspace/AtomSpace.cc
 *
 * Copyright (C) 2008-2011 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include "AtomSpace.h"

#include <stdlib.h>
#include <string>

#include <opencog/util/Logger.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

// ====================================================================
//

AtomSpace::AtomSpace(void)
{
    _atomSpaceImpl = new AtomSpaceImpl();
    _ownsAtomSpaceImpl = true;

    c_add = addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::AtomSpace(const AtomSpace& other)
{
    _atomSpaceImpl = other._atomSpaceImpl;
    _ownsAtomSpaceImpl = false;

    c_add = addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::AtomSpace(AtomSpaceImpl* a)
{
    _atomSpaceImpl = a;
    _ownsAtomSpaceImpl = false;

    c_add = addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::~AtomSpace()
{
    c_add.disconnect();
    // Will be unnecessary once GC is implemented
    if (_ownsAtomSpaceImpl)
        delete _atomSpaceImpl;
}

bool AtomSpace::handleAddSignal(Handle h)
{
    // XXX TODO FIXME  this must be locked to avoid corruption!!!
    addAtomSignalQueue.push_back(h);
    return false;
}

AtomSpace& AtomSpace::operator=(const AtomSpace& other)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpace - Cannot copy an object of this class");
}

Handle AtomSpace::addPrefixedNode(Type t, const std::string& prefix, TruthValuePtr tvn)
{
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    static unsigned long int cnt = 0;
    srand(++cnt);
    static const int len = 16;
    std::string name;
    Handle result;
    //Keep trying new random suffixes until you generate a new name
    do {
        name = prefix;
        for (int i = 0; i < len; ++i) {
            name += alphanum[rand() % (sizeof(alphanum) - 1)];
        }
        result = getHandle(t, name);
    } while (isValidHandle(result)); // If the name already exists, try again
    return addNode(t, name, tvn);
}

