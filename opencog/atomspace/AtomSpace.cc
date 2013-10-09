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

#include <pthread.h>
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
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    // Ensure caching is ready before anything else happens
    setUpCaching();
#endif
    atomSpaceAsync = new AtomSpaceAsync();
    ownsAtomSpaceAsync = true;
    c_remove = atomSpaceAsync->removeAtomSignal(
            boost::bind(&AtomSpace::atomRemoveSignal, this, _1, _2));
    c_add = atomSpaceAsync->addAtomSignal(
            boost::bind(&AtomSpace::handleAddSignal, this, _1, _2));
}

AtomSpace::AtomSpace(const AtomSpace& other)
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    // Ensure caching is ready before anything else happens
    setUpCaching();
#endif
    this->atomSpaceAsync = other.atomSpaceAsync;
    ownsAtomSpaceAsync = false;
    c_remove = atomSpaceAsync->removeAtomSignal(
            boost::bind(&AtomSpace::atomRemoveSignal, this, _1, _2));
    c_add = atomSpaceAsync->addAtomSignal(
            boost::bind(&AtomSpace::handleAddSignal, this, _1, _2));
}


#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
void AtomSpace::setUpCaching()
{
    // Initialise lru cache for getType
    __getType = new _getType(this);
    getTypeCached = new lru_cache_threaded<AtomSpace::_getType>(1000, *__getType);

}
#endif

AtomSpace::AtomSpace(AtomSpaceAsync& a)
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    setUpCaching();
#endif
    atomSpaceAsync = &a;
    ownsAtomSpaceAsync = false;
}

AtomSpace::~AtomSpace()
{
    c_remove.disconnect();
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    delete __getType;
    delete getTypeCached;
#endif
    // Will be unnecessary once GC is implemented
    if (ownsAtomSpaceAsync)
        delete atomSpaceAsync;
}

bool AtomSpace::handleAddSignal(AtomSpaceImpl *as, Handle h)
{
    addAtomSignalQueue.push_back(h);
    return false;
}

bool AtomSpace::atomRemoveSignal(AtomSpaceImpl *as, AtomPtr a)
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    getTypeCached->remove(a->getHandle());
#endif
    return false;
}

Type AtomSpace::getType(Handle h) const
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    Type t = (*getTypeCached)(h);
    return t;
#else
    return atomSpaceAsync->getType(h)->get_result();
#endif
}

strength_t AtomSpace::getMean(Handle h, VersionHandle vh) const
{
    FloatRequest tvr = atomSpaceAsync->getMean(h, vh);
    return tvr->get_result();
}

confidence_t AtomSpace::getConfidence(Handle h, VersionHandle vh) const
{
    FloatRequest tvr = atomSpaceAsync->getConfidence(h, vh);
    return tvr->get_result();
}

/*tv_summary_t AtomSpace::getTV(Handle h, VersionHandle vh) const
{
    TruthValueRequest tvr = atomSpaceAsync->getTV(h, vh);
    return tvr->get_result();
}*/

TruthValuePtr AtomSpace::getTV(Handle h, VersionHandle vh) const
{
    TruthValueCompleteRequest tvr = atomSpaceAsync->getTVComplete(h, vh);
    TruthValuePtr x(tvr->get_result());
    tvr->result = NULL; // cheat to avoid copying TruthValue once again
    return x;
}

void AtomSpace::setTV(Handle h, const TruthValue& tv, VersionHandle vh)
{
    atomSpaceAsync->setTV(h, tv, vh)->get_result();
}

AtomSpace& AtomSpace::operator=(const AtomSpace& other)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpace - Cannot copy an object of this class");
}

bool AtomSpace::isNode(const Handle& h) const
{
    DPRINTF("AtomSpace::isNode Atom space address: %p\n", this);
    Type t = getType(h);
    return classserver().isA(t, NODE);
}

bool AtomSpace::isLink(const Handle& h) const
{
    DPRINTF("AtomSpace::isLink Atom space address: %p\n", this);
    Type t = getType(h);
    return classserver().isA(t, LINK);
}

void AtomSpace::do_merge_tv(Handle h, const TruthValue& tvn)
{
    TruthValuePtr currentTV(getTV(h));
    if (currentTV->isNullTv()) {
        setTV(h, tvn);
    } else {
        TruthValue* mergedTV = currentTV->merge(tvn);
        setTV(h, *mergedTV);
        delete mergedTV;
    }
}

Handle AtomSpace::addPrefixedNode(Type t, const string& prefix, const TruthValue& tvn) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    srand(time(0));
    static const int len = 16;
    string name;
    Handle result;
    //Keep trying new random suffixes until you generate a new name
    do {
        name=prefix;
        for (int i = 0; i < len; ++i) {
            name+=alphanum[rand() % (sizeof(alphanum) - 1)];
        }
        result = getHandle(t, name);
    } while(isValidHandle(result));//If the name already exists, try again
    return addNode(t, name, tvn);
}

bool AtomSpace::isValidHandle(const Handle& h) const
{
    return atomSpaceAsync->isValidHandle(h)->get_result();
}

AttentionValue AtomSpace::getAV(Handle h) const
{
    return atomSpaceAsync->getAV(h)->get_result();
}

void AtomSpace::setAV(Handle h, const AttentionValue &av)
{
    atomSpaceAsync->setAV(h,av)->get_result();
}

int AtomSpace::Nodes(VersionHandle vh) const
{
    return atomSpaceAsync->nodeCount(vh)->get_result();
}

AttentionValue::sti_t AtomSpace::getAttentionalFocusBoundary() const
{
    return atomSpaceAsync->atomspace.getAttentionBank().getAttentionalFocusBoundary();
}

int AtomSpace::Links(VersionHandle vh) const
{
    return atomSpaceAsync->linkCount(vh)->get_result();
}

void AtomSpace::decayShortTermImportance()
{
    atomSpaceAsync->decayShortTermImportance()->get_result();
}

AttentionValue::sti_t AtomSpace::setAttentionalFocusBoundary(AttentionValue::sti_t s)
{
    return atomSpaceAsync->atomspace.getAttentionBank().setAttentionalFocusBoundary(s);
}

void AtomSpace::clear()
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    {
        getTypeCached->clear();
    }
#endif
    atomSpaceAsync->clear()->get_result();
}

void AtomSpace::print(std::ostream& output,
           Type type, bool subclass) const
{
    atomSpaceAsync->print(output, type, subclass)->get_result();
}


bool AtomSpace::isHandleInSeq(Handle h, HandleSeq &seq)
{
    HandleSeq::const_iterator it;
    for (it = seq.begin(); it != seq.end(); ++ it)
    {
        if ((Handle)(*it) == h)
            return true;
    }

    return false;
}

