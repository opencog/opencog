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
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/types.h>
#include <opencog/persist/xml/NMXmlExporter.h>
#include <opencog/util/Config.h>
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
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    setUpCaching();
#endif
}


#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
void AtomSpace::setUpCaching() 
{
    // Initialise lru cache for getType
    __getType = new _getType(this);
    getTypeCached = new lru_cache<AtomSpace::_getType>(1000,*__getType);

    // Initialise lru cache for getTV
    //__getTV = new _getTV(this);
    //getTVCached = new lru_cache<AtomSpace::_getTV>(1000,*__getTV);

    c_remove = atomSpaceAsync->removeAtomSignal(
            boost::bind(&AtomSpace::handleRemoveSignal, this, _1, _2));

}
#endif

AtomSpace::AtomSpace(AtomSpaceAsync& a)
{
    atomSpaceAsync = &a;
    ownsAtomSpaceAsync = false;
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    setUpCaching();
#endif

}

AtomSpace::~AtomSpace()
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    c_remove.disconnect();
    boost::mutex::scoped_lock(cache_lock);
    delete __getType;
    delete getTypeCached;
#endif
    // Will be unnecessary once GC is implemented
    if (ownsAtomSpaceAsync)
        delete atomSpaceAsync;
}

#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
bool AtomSpace::handleRemoveSignal(AtomSpaceImpl *as, Handle h)
{
    boost::mutex::scoped_lock(cache_lock);
    getTypeCached->make_dirty(h);
    return false;
}
#endif

Type AtomSpace::getType(Handle h) const
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    boost::mutex::scoped_lock(cache_lock);
    return (*getTypeCached)(h);
#else
    return atomSpaceAsync->getType(h)->get_result();
#endif
}

const TruthValue* AtomSpace::getTV(Handle h, VersionHandle vh) const
{
    TruthValueRequest tvr = atomSpaceAsync->getTV(h, vh);
    return tvr->get_result();
    //const TruthValue& result = *tvr->get_result();
    // Need to clone the result's TV as it will be deleted when the request
    // is.
    //return TruthValue*(result.clone());
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

AtomSpace::AtomSpace(const AtomSpace& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomSpace - Cannot copy an object of this class");
}

bool AtomSpace::inheritsType(Type t1, Type t2) const
{
    DPRINTF("AtomSpace::inheritsType Atom space address: %p\n", this);
    DPRINTF("AtomSpace::inheritsType(t1=%d,t2=%d)\n", t1, t2);
    bool result = classserver().isA(t1, t2);
    DPRINTF("AtomSpace::inheritsType result = %d\n", result);
    return result;
}

bool AtomSpace::isNode(Type t) const
{
    DPRINTF("AtomSpace::isNode Atom space address: %p\n", this);
    return inheritsType(t, NODE);
}

bool AtomSpace::isLink(Type t) const
{
    DPRINTF("AtomSpace::isLink Atom space address: %p\n", this);
    return inheritsType(t, LINK);
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

string AtomSpace::getName(Type t) const
{
    DPRINTF("AtomSpace::getName Atom space address: %p\n", this);
    return classserver().getTypeName(t);
}

void AtomSpace::do_merge_tv(Handle h, const TruthValue& tvn)
{
    const TruthValue* currentTV(getTV(h));
    if (currentTV->isNullTv()) {
        setTV(h, tvn);
    } else {
        TruthValue* mergedTV = currentTV->merge(tvn);
        setTV(h, *mergedTV);
        delete mergedTV;
    }
}

Handle AtomSpace::addNode(Type t, const string& name, const TruthValue& tvn)
{
    Handle result = getHandle(t, name);
    if (TLB::isValidHandle(result))
    {
        // Just merges the TV
        // if (!tvn.isNullTv()) do_merge_tv(result, tvn);
        // Even if the node already exists, it must be merged properly 
        // for updating its truth and attention values. 
        atomTable.merge(result, tvn); 
        return result;
    }

    // Remove default STI/LTI from AtomSpace Funds
    fundsSTI -= AttentionValue::DEFAULTATOMSTI;
    fundsLTI -= AttentionValue::DEFAULTATOMLTI;

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        Node *n = backing_store->getNode(t, name.c_str());
        if (n)
        {
            result = TLB::getHandle(n);
            // TODO: Check if merge signal must be emitted here (AtomTable::merge
            // does that, but what to do with atoms that are not there?)
            if (!tvn.isNullTv()) do_merge_tv(result, tvn);
            return atomTable.add(n);
        }
    }

    return atomTable.add(new Node(t, name, tvn));
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
        //std::stringstream out;
        //out << prefix;
        for (int i = 0; i < len; ++i) {
            name+=alphanum[rand() % (sizeof(alphanum) - 1)];
            //out << alphanum[rand() % (sizeof(alphanum) - 1)];
        }
        //name = out.str();
        result = getHandle(t, name);
    } while(TLB::isValidHandle(result));

    return addNode(t, name, tvn);
}
  


Handle AtomSpace::addLink(Type t, const HandleSeq& outgoing,
                          const TruthValue& tvn)
{
    Handle result = getHandle(t, outgoing);
    if (TLB::isValidHandle(result))
    {
        // Just merges the TV
        //if (!tvn.isNullTv()) do_merge_tv(result, tvn);
        // Even if the node already exists, it must be merged properly 
        // for updating its truth and attention values. 
        atomTable.merge(result, tvn); 
        return result;
    }

    // Remove default STI/LTI from AtomSpace Funds
    fundsSTI -= AttentionValue::DEFAULTATOMSTI;
    fundsLTI -= AttentionValue::DEFAULTATOMLTI;

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        Link *l = backing_store->getLink(t, outgoing);
        if (l)
        {
            result = TLB::getHandle(l);
            // TODO: Check if merge signal must be emitted here (AtomTable::merge
            // does that, but what to do with atoms that are not there?)
            if (!tvn.isNullTv()) do_merge_tv(result, tvn);
            return atomTable.add(l);
        }
    }

    return atomTable.add(new Link(t, outgoing, tvn));
}

Handle AtomSpace::fetchAtom(Handle h)
{
    // No-op if we've already got this handle.
    // XXX But perhaps we want to update the truth value from the
    // remote storage?? XXX the semantics of this is totally unclear.
    if (atomTable.holds(h)) return h;

    // If its in the TLB, but not in the atom table, insert it now.
    if (TLB::isValidHandle(h))
    {
        Atom *a = TLB::getAtom(h);

        // For links, must perform a recursive fetch, as otherwise
        // the atomtable.add below will throw an error.
        Link *l = dynamic_cast<Link *>(a);
        if (l)
        {
           const std::vector<Handle>& ogs = l->getOutgoingSet();
           size_t arity = ogs.size();
           for (size_t i=0; i<arity; i++)
           {
              Handle oh = fetchAtom(ogs[i]);
              if (oh != ogs[i])
              {
                  logger().info(
                      "Unexpected handle mismatch:\n"
                      "oh=%lu ogs[%d]=%lu\n",
                      oh.value(), i, ogs[i].value());

                  Atom *ah = TLB::getAtom(oh);
                  Atom *ag = TLB::getAtom(ogs[i]);
                  if (ah) logger().info("Atom oh: %s\n",
                      ah->toString().c_str());
                  else logger().info("Atom oh: (null)\n");

                  if (ag) logger().info("Atom ogs[i]: %s\n",
                      ag->toString().c_str());
                  else logger().info("Atom ogs[i]: (null)\n");

                  throw new RuntimeException(TRACE_INFO,
                      "Unexpected handle mismatch\n");
              }
           }
        }
        return atomTable.add(a);
    }

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        Atom *a = backing_store->getAtom(h);

        // For links, must perform a recursive fetch, as otherwise
        // the atomtable.add below will throw an error.
        Link *l = dynamic_cast<Link *>(a);
        if (l)
        {
           const std::vector<Handle>& ogs = l->getOutgoingSet();
           size_t arity = ogs.size();
           for (size_t i=0; i<arity; i++)
           {
              Handle oh = fetchAtom(ogs[i]);
              if (oh != ogs[i]) throw new RuntimeException(TRACE_INFO,
                    "Unexpected handle mismatch -B!\n");
           }
        }
        if (a) return atomTable.add(a);
    }
    
    return Handle::UNDEFINED;
}

Handle AtomSpace::fetchIncomingSet(Handle h, bool recursive)
{
    Handle base = fetchAtom(h);
    if (Handle::UNDEFINED == base) return Handle::UNDEFINED;

    // Get everything from the backing store.
    if (backing_store)
    {
        std::vector<Handle> iset = backing_store->getIncomingSet(h);
        size_t isz = iset.size();
        for (size_t i=0; i<isz; i++)
        {
            Handle hi = iset[i];
            if (recursive)
            {
                fetchIncomingSet(hi, true);
            }
            else
            {
                fetchAtom(hi);
            }
        }
    }
    return base;
}

Handle AtomSpace::addRealAtom(const Atom& atom, const TruthValue& tvn)
{
    DPRINTF("AtomSpace::addRealAtom\n");
    const TruthValue& newTV = (tvn.isNullTv()) ? atom.getTruthValue() : tvn;
    // Check if the given Atom reference is of an atom
    // that was not inserted yet.  If so, adds the atom. Otherwise, just sets
    // result to the correct/valid handle.
    Handle result;
    const Node *node = dynamic_cast<const Node *>(&atom);
    if (node) {
        result = getHandle(node->getType(), node->getName());
        if (result == Handle::UNDEFINED) {
            return addNode(node->getType(), node->getName(), newTV);
        }
    } else {
        const Link *link = dynamic_cast<const Link *>(&atom);
        result = getHandle(link->getType(), link->getOutgoingSet());
        if (result == Handle::UNDEFINED) {
            return addLink(link->getType(), link->getOutgoingSet(), newTV);
        }
    }
    do_merge_tv(result,newTV);
    return result;
}

boost::shared_ptr<Atom> AtomSpace::cloneAtom(const Handle h) const
{
    return atomSpaceAsync->getAtom(h)->get_result();
}

size_t AtomSpace::getAtomHash(const Handle h) const 
{
    return atomSpaceAsync->getAtomHash(h)->get_result();
}

bool AtomSpace::isValidHandle(const Handle h) const 
{
    return atomSpaceAsync->isValidHandle(h)->get_result();
}

bool AtomSpace::commitAtom(const Atom& a)
{
    return atomSpaceAsync->commitAtom(a)->get_result();
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

int AtomSpace::Links(VersionHandle vh) const
{
    return atomSpaceAsync->linkCount(vh)->get_result();
}

void AtomSpace::decayShortTermImportance()
{
    atomSpaceAsync->decayShortTermImportance()->get_result();
}

long AtomSpace::getTotalSTI() const
{
    long totalSTI = 0;
    HandleSeq hs;

    getHandleSet(back_inserter(hs), ATOM, true);
    foreach (Handle h, hs) totalSTI += getSTI(h);
    return totalSTI;

}

long AtomSpace::getTotalLTI() const
{
    long totalLTI = 0;
    HandleSeq hs;

    getHandleSet(back_inserter(hs), ATOM, true);
    foreach (Handle h, hs) totalLTI += getLTI(h);
    return totalLTI;
}

AttentionValue::sti_t AtomSpace::getAttentionalFocusBoundary() const
{
    return atomSpaceAsync->atomspace.getAttentionBank().getAttentionalFocusBoundary();
}

AttentionValue::sti_t AtomSpace::setAttentionalFocusBoundary(AttentionValue::sti_t s)
{
    return atomSpaceAsync->atomspace.getAttentionBank().setAttentionalFocusBoundary(s);
}

void AtomSpace::clear()
{
#ifdef USE_ATOMSPACE_LOCAL_THREAD_CACHE
    {
        boost::mutex::scoped_lock(cache_lock);
        getTypeCached->clear();
        //getTVCached->clear();
    }
#endif
    atomSpaceAsync->clear()->get_result();
}

void AtomSpace::print(std::ostream& output,
           Type type, bool subclass) const
{
    atomSpaceAsync->print(output, type, subclass)->get_result();
}
