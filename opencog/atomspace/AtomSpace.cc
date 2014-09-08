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

#include <stdlib.h>
#include <string>

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include "AtomSpace.h"

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

// ====================================================================
//

AtomSpace::AtomSpace(AtomSpace* parent)
{
    _atomSpaceImpl = new AtomSpaceImpl(parent? parent->_atomSpaceImpl : NULL);
}

AtomSpace::~AtomSpace()
{
    delete _atomSpaceImpl;
}

AtomSpace& AtomSpace::operator=(const AtomSpace& other)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpace - Cannot copy an object of this class");
}

AtomSpace::AtomSpace(const AtomSpace& other)
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

