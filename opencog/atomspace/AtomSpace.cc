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

#include <string>
#include <iostream>
#include <fstream>
#include <list>
#include <time.h>
#include <cstdlib>

#include <stdlib.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

//#define DPRINTF printf
#define DPRINTF(...)

using std::string;
using std::cerr;
using std::cout;
using std::endl;
using std::min;
using std::max;
using namespace opencog;

// ====================================================================
//

AtomSpace::AtomSpace(void)
{
    atomSpaceAsync = new AtomSpaceAsync();
    ownsAtomSpaceAsync = true;

    c_add = atomSpaceAsync->addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::AtomSpace(const AtomSpace& other)
{
    this->atomSpaceAsync = other.atomSpaceAsync;
    ownsAtomSpaceAsync = false;

    c_add = atomSpaceAsync->addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::AtomSpace(AtomSpaceAsync& a)
{
    atomSpaceAsync = &a;
    ownsAtomSpaceAsync = false;

    c_add = atomSpaceAsync->addAtomSignal(
        boost::bind(&AtomSpace::handleAddSignal, this, _1));
}

AtomSpace::~AtomSpace()
{
    c_add.disconnect();
    // Will be unnecessary once GC is implemented
    if (ownsAtomSpaceAsync)
        delete atomSpaceAsync;
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

Handle AtomSpace::addPrefixedNode(Type t, const string& prefix, TruthValuePtr tvn)
{
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    static unsigned long int cnt = 0;
    srand(++cnt);
    static const int len = 16;
    string name;
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

