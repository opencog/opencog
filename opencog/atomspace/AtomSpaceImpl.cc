/*
 * opencog/atomspace/AtomSpaceImpl.cc
 *
 * Copyright (C) 2010 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@opencog.org>
 *            Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
 *            Welter Silva <welter@vettalabs.com>
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

#include "AtomSpaceImpl.h"

#include <string>
#include <iostream>
#include <fstream>
#include <list>

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
#include <opencog/atomspace/TLB.h>
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

AtomSpaceImpl::AtomSpaceImpl(void) : 
    type_itr(0,false)
{
    _handle_iterator = NULL;
    emptyName = "";
    backing_store = NULL;
    spaceServer = NULL;

    fundsSTI = config().get_int("STARTING_STI_FUNDS");
    fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;

    //connect signals
    addedAtomConnection = addAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomAdded, this, _1, _2));
    removedAtomConnection = removeAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomRemoved, this, _1, _2));

    logger().fine("Max load factor for handle map is: %f", TLB::handle_map.max_load_factor());

    pthread_mutex_init(&atomSpaceLock, NULL);
}

AtomSpaceImpl::~AtomSpaceImpl()
{
    //disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();

    // Check if has already been deleted. See in code where it can be delete.
    if (_handle_iterator) {
        delete _handle_iterator;
         _handle_iterator = NULL;
    }
}

// ====================================================================

void AtomSpaceImpl::registerBackingStore(BackingStore *bs)
{
    backing_store = bs;
}

void AtomSpaceImpl::unregisterBackingStore(BackingStore *bs)
{
    if (bs == backing_store) backing_store = NULL;
}

// ====================================================================

void AtomSpaceImpl::atomAdded(AtomSpaceImpl *a, Handle h)
{
    DPRINTF("AtomSpaceImpl::atomAdded(%p): %s\n", h, TLB::getAtom(h)->toString().c_str());
    Type type = getType(h);
    if (type == CONTEXT_LINK) {
        // Add corresponding VersionedTV to the contextualized atom
        // Note that when a VersionedTV is added to a
        // CompositeTruthValue it will not automatically add a
        // corresponding ContextLink
        if (getArity(h) == 2) {
            Handle cx = getOutgoing(h, 0); // context
            Handle ca = getOutgoing(h, 1); // contextualized atom
            setTV(ca, getTV(h), VersionHandle(CONTEXTUAL, cx));
        } else logger().warn("AtomSpaceImpl::atomAdded: Invalid arity for a ContextLink: %d (expected: 2)\n", getArity(h));
    }
}

void AtomSpaceImpl::atomRemoved(AtomSpaceImpl *a, Handle h)
{
    Type type = getType(h);
    if (type == CONTEXT_LINK) {
        // Remove corresponding VersionedTV to the contextualized atom
        // Note that when a VersionedTV is removed from a
        // CompositeTruthValue it will not automatically remove the
        // corresponding ContextLink
        OC_ASSERT(getArity(h) == 2, "AtomSpaceImpl::atomRemoved: Got invalid arity for removed ContextLink = %d\n", getArity(h));
        Handle cx = getOutgoing(h, 0); // context
        Handle ca = getOutgoing(h, 1); // contextualized atom
        const TruthValue& tv = getTV(ca);
        OC_ASSERT(tv.getType() == COMPOSITE_TRUTH_VALUE);
        CompositeTruthValue new_ctv(static_cast<const CompositeTruthValue&>(tv));
        new_ctv.removeVersionedTV(VersionHandle(CONTEXTUAL, cx));
        // @todo: one may want improve that code by converting back
        // the CompositeTV into a simple or indefinite TV when it has
        // no more VersionedTV
        setTV(ca, new_ctv);
    } 
    else if ( inheritsType(type, OBJECT_NODE) ) {
        // TODO remove this and we can remove spaceServer from the
        // AtomSpaceImpl class
        spaceServer->removeObject(getName(h));
    } // else if
}

// ====================================================================

const AtomTable& AtomSpaceImpl::getAtomTable() const
{
    DPRINTF("AtomSpaceImpl::getAtomTable Atom space address: %p\n", this);
    return atomTable;
}

void AtomSpaceImpl::print(std::ostream& output, Type type, bool subclass) const
{
    atomTable.print(output, type, subclass);
}


AtomSpaceImpl& AtomSpaceImpl::operator=(const AtomSpaceImpl& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomSpaceImpl - Cannot copy an object of this class");
}

AtomSpaceImpl::AtomSpaceImpl(const AtomSpaceImpl& other):
    type_itr(0,false)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomSpaceImpl - Cannot copy an object of this class");
}

const TruthValue& AtomSpaceImpl::getDefaultTV()
{
    return TruthValue::DEFAULT_TV();
}

Type AtomSpaceImpl::getType(Handle h) const
{
    DPRINTF("AtomSpaceImpl::getType Atom space address: %p\n", this);
    Atom* a = TLB::getAtom(h);
    if (a) return a->getType();
    else return NOTYPE;
}

Type AtomSpaceImpl::getAtomType(const string& str) const
{
    DPRINTF("AtomSpaceImpl::getAtomType Atom space address: %p\n", this);

    return classserver().getType(const_cast<char*>(str.c_str()));
}

bool AtomSpaceImpl::inheritsType(Type t1, Type t2) const
{
    DPRINTF("AtomSpaceImpl::inheritsType Atom space address: %p\n", this);
    DPRINTF("AtomSpaceImpl::inheritsType(t1=%d,t2=%d)\n", t1, t2);
    bool result = classserver().isA(t1, t2);
    DPRINTF("AtomSpaceImpl::inheritsType result = %d\n", result);
    return result;
}

bool AtomSpaceImpl::isNode(Type t) const
{
    DPRINTF("AtomSpaceImpl::isNode Atom space address: %p\n", this);
    return inheritsType(t, NODE);
}

bool AtomSpaceImpl::isLink(Type t) const
{
    DPRINTF("AtomSpaceImpl::isLink Atom space address: %p\n", this);
    return inheritsType(t, LINK);
}

bool AtomSpaceImpl::isNode(const Handle& h) const
{
    DPRINTF("AtomSpaceImpl::isNode Atom space address: %p\n", this);
    Type t = getType(h);
    return classserver().isA(t, NODE);
}

bool AtomSpaceImpl::isLink(const Handle& h) const
{
    DPRINTF("AtomSpaceImpl::isLink Atom space address: %p\n", this);
    Type t = getType(h);
    return classserver().isA(t, LINK);
}

string AtomSpaceImpl::getName(Type t) const
{
    DPRINTF("AtomSpaceImpl::getName Atom space address: %p\n", this);
    return classserver().getTypeName(t);
}

bool AtomSpaceImpl::isVar(Handle h) const
{
    DPRINTF("AtomSpaceImpl::isVar Atom space address: %p\n", this);
    return inheritsType(getType(h), VARIABLE_NODE);
}

bool AtomSpaceImpl::isList(Handle h) const
{
    DPRINTF("AtomSpaceImpl::isList Atom space address: %p\n", this);
    return inheritsType(getType(h), LIST_LINK);
}

bool AtomSpaceImpl::containsVar(Handle h) const
{
    DPRINTF("AtomSpaceImpl::containsVar Atom space address: %p\n", this);

    Node *nnn = dynamic_cast<Node *>(TLB::getAtom(h));
    if (!nnn) {
        for (int i = 0;i < getArity(h);++i)
            if (containsVar(getOutgoing(h, i)))
                return true;
        return false;
    }
    return isVar(h);
}

Handle AtomSpaceImpl::createHandle(Type t, const string& str, bool managed)
{
    DPRINTF("AtomSpaceImpl::createHandle Atom space address: %p\n", this);

    Handle h = getHandle(t, str);
    return TLB::isValidHandle(h) ? h : addNode(t, str, TruthValue::NULL_TV());
}

Handle AtomSpaceImpl::createHandle(Type t, const HandleSeq& outgoing, bool managed)
{
    DPRINTF("AtomSpaceImpl::createHandle Atom space address: %p\n", this);

    Handle h = getHandle(t, outgoing);
    return TLB::isValidHandle(h) ? h : addLink(t, outgoing, TruthValue::NULL_TV());
}

bool AtomSpaceImpl::containsVersionedTV(Handle h, VersionHandle vh) const
{
    DPRINTF("AtomSpaceImpl::containsVersionedTV Atom space address: %p\n", this);

    bool result = isNullVersionHandle(vh);
    if (!result) {
        const TruthValue& tv = this->getTV(h);
        result = !tv.isNullTv() && tv.getType() == COMPOSITE_TRUTH_VALUE &&
                 !(((const CompositeTruthValue&) tv).getVersionedTV(vh).isNullTv());
    }
    return result;
}

bool AtomSpaceImpl::removeAtom(Handle h, bool recursive)
{
    HandleEntry* extractedHandles = atomTable.extract(h, recursive);
    if (extractedHandles) {
        HandleEntry* currentEntry = extractedHandles;
        while (currentEntry) {
            Handle h = currentEntry->handle;

            // Also refund sti/lti to AtomSpace funds pool
            fundsSTI += getSTI(h);
            fundsLTI += getLTI(h);

            // emit remove atom signal
            _removeAtomSignal(this,h);

            currentEntry = currentEntry->next;
        }
        atomTable.removeExtractedHandles(extractedHandles);
        
        delete extractedHandles;
        return true;
    }
    return false;
}

const HandleSeq& AtomSpaceImpl::getOutgoing(Handle h) const
{
    static HandleSeq hs;
    Link *link = dynamic_cast<Link *>(TLB::getAtom(h));
    if (!link) return hs;
    return link->getOutgoingSet();
}

void AtomSpaceImpl::do_merge_tv(Handle h, const TruthValue& tvn)
{
    const TruthValue& currentTV = getTV(h);
    if (currentTV.isNullTv()) {
        setTV(h, tvn);
    } else {
        TruthValue* mergedTV = currentTV.merge(tvn);
        setTV(h, *mergedTV);
        delete mergedTV;
    }
}

Handle AtomSpaceImpl::addNode(Type t, const string& name, const TruthValue& tvn)
{
    Handle result = getHandle(t, name);
    if (TLB::isValidHandle(result))
    {
        // Just merges the TV
        // if (!tvn.isNullTv()) do_merge_tv(result, tvn);
        // Even if the node already exists, it must be merged properly 
        // for updating its truth and attention values. 
        atomTable.merge(result, tvn); 
        // emit "merge atom" signal
        _mergeAtomSignal(this,result);
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

    Handle newNodeHandle = atomTable.add(new Node(t, name, tvn));
    // emit add atom signal
    _addAtomSignal(this,newNodeHandle);
    return newNodeHandle;
}

Handle AtomSpaceImpl::addLink(Type t, const HandleSeq& outgoing,
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
        _mergeAtomSignal(this,result);
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

    Handle newLinkHandle = atomTable.add(new Link(t, outgoing, tvn));
    // emit add atom signal
    _addAtomSignal(this,newLinkHandle);
    return newLinkHandle;
}

Handle AtomSpaceImpl::fetchAtom(Handle h)
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

Handle AtomSpaceImpl::fetchIncomingSet(Handle h, bool recursive)
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

Handle AtomSpaceImpl::addRealAtom(const Atom& atom, const TruthValue& tvn)
{
    DPRINTF("AtomSpaceImpl::addRealAtom\n");
    const TruthValue& newTV = (tvn.isNullTv()) ? atom.getTruthValue() : tvn;
    // Check if the given Atom reference is of an atom
    // that was not inserted yet.  If so, adds the atom. Otherwise, just sets
    // result to the correct/valid handle.
    Handle result;
    const Node *node = dynamic_cast<const Node *>(&atom);
    if (node) {
        result = getHandle(node->getType(), node->getName());
        if (TLB::isInvalidHandle(result)) {
            return addNode(node->getType(), node->getName(), newTV);
        }
    } else {
        const Link *link = dynamic_cast<const Link *>(&atom);
        result = getHandle(link->getType(), link->getOutgoingSet());
        if (TLB::isInvalidHandle(result)) {
            return addLink(link->getType(), link->getOutgoingSet(), newTV);
        }
    }
    const TruthValue& currentTV = getTV(result);
    if (currentTV.isNullTv()) {
        setTV(result, newTV);
    } else {
        TruthValue* mergedTV = currentTV.merge(newTV);
        setTV(result, *mergedTV);
        delete mergedTV;
    }

    return result;
}

const string& AtomSpaceImpl::getName(Handle h) const
{
    Node * nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    if (nnn)
        return nnn->getName();
    else
        return emptyName;
}

boost::shared_ptr<Atom> AtomSpaceImpl::cloneAtom(const Handle h) const
{
    // TODO: Add timestamp to atoms and add vector clock to AtomSpace
    // Need to use the newly added clone methods as the copy constructors for
    // Node and Link don't copy incoming set.
    Atom * a = TLB::getAtom(h);
    boost::shared_ptr<Atom> dud;
    if (!a) return dud;
    const Node *node = dynamic_cast<const Node *>(a);
    if (!node) {
        const Link *l = dynamic_cast<const Link *>(a);
        if (!l) return dud;
        boost::shared_ptr<Atom> clone_link(l->clone());
        return clone_link;
    } else {
        boost::shared_ptr<Atom> clone_node(node->clone());
        return clone_node;
    }
}

boost::shared_ptr<Node> AtomSpaceImpl::cloneNode(const Handle h) const
{
    return boost::shared_dynamic_cast<Node>(this->cloneAtom(h));
}

boost::shared_ptr<Link> AtomSpaceImpl::cloneLink(const Handle h) const
{
    return boost::shared_dynamic_cast<Link>(this->cloneAtom(h));
}

std::string AtomSpaceImpl::atomAsString(Handle h, bool terse) const
{
    // TODO check that h is a valid atom handle
    if (terse) return TLB::getAtom(h)->toShortString();
    return TLB::getAtom(h)->toString();
}

HandleSeq AtomSpaceImpl::getNeighbors(const Handle h, bool fanin,
        bool fanout, Type desiredLinkType, bool subClasses) const 
{
    Atom* a = TLB::getAtom(h);
    if (a == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Handle %d doesn't refer to a Atom", h.value());
    }
    HandleSeq answer;

    for (HandleEntry *he = a->getIncomingSet(); he != NULL; he = he ->next) {
        Link *link = dynamic_cast<Link*>(TLB::getAtom(he->handle));
        Type linkType = link->getType();
        DPRINTF("Atom::getNeighbors(): linkType = %d desiredLinkType = %d\n", linkType, desiredLinkType);
        if ((linkType == desiredLinkType) || (subClasses && classserver().isA(linkType, desiredLinkType))) {
            int linkArity = link->getArity();
            for (int i = 0; i < linkArity; i++) {
                Handle handle = link->getOutgoingSet()[i];
                if (handle == h) continue;
                if (!fanout && link->isSource(h)) continue;
                if (!fanin && link->isTarget(h)) continue;
                answer.push_back(handle);
            }
        }
    }
    return answer;
}

bool AtomSpaceImpl::isSource(Handle source, Handle link) const
{
    Atom *a = TLB::getAtom(link);
    const Link *l = dynamic_cast<const Link *>(a);
    if (l != NULL) {
        return l->isSource(source);
    }
    return false;
}

size_t AtomSpaceImpl::getAtomHash(const Handle h) const 
{
    return TLB::getAtom(h)->hashCode();
}

bool AtomSpaceImpl::isValidHandle(const Handle h) const 
{
    return TLB::isValidHandle(h);
}

bool AtomSpaceImpl::commitAtom(const Atom& a)
{
    // TODO: Check for differences and abort if timestamp is out of date

    // Get the version in the TLB
    Atom* original = TLB::getAtom(a.getHandle());
    // The only mutable properties of atoms are the TV and AttentionValue
    // TODO: this isn't correct, trails, flags and other things might change
    // too...
    original->setTruthValue(a.getTruthValue());
    original->setAttentionValue(a.getAttentionValue());
    return true;
}

Handle AtomSpaceImpl::getOutgoing(Handle h, int idx) const
{
    Atom * a = TLB::getAtom(h);
    Link * l = dynamic_cast<Link *>(a);
    if (NULL == l)
        throw new IndexErrorException(TRACE_INFO,
            "Asked for outgoing set on atom that is not a link!\n");
    if (idx >= l->getArity())
        throw new IndexErrorException(TRACE_INFO, "Invalid outgoing set index: %d (atom: %s)\n",
                                      idx, a->toString().c_str());
    return l->getOutgoingSet()[idx];
}

int AtomSpaceImpl::getArity(Handle h) const
{
    Atom * a = TLB::getAtom(h);
    Link * l = dynamic_cast<Link *>(a);
    if (NULL == l)
        return 0;
    return l->getArity();
}

void AtomSpaceImpl::setName(Handle h, const string& name)
{
    Node *nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    OC_ASSERT(nnn != NULL, "AtomSpaceImpl::setName(): Handle h should be of 'Node' type.");
    nnn->setName(name);
}

HandleSeq AtomSpaceImpl::getIncoming(Handle h)
{
    // It is possible that the incoming set that we currently 
    // hold is much smaller than what is in storage. In this case,
    // we would like to automatically pull all of those other atoms
    // into here (using fetchIncomingSet(h,true) to do so). However,
    // maybe the incoming set is up-to-date, in which case polling 
    // storage over and over is a huge waste of time.  What to do? 
    //
    // h = fetchIncomingSet(h, true);
    //
    // TODO: solution where user can specify whether to poll storage/repository

    HandleEntry* he = TLB::getAtom(h)->getIncomingSet();
    if (he) return he->toHandleVector();
    else return HandleSeq();
}

void AtomSpaceImpl::setTV(Handle h, const TruthValue& tv, VersionHandle vh)
{
    const TruthValue& currentTv = getTV(h);
    if (!isNullVersionHandle(vh))
    {
        CompositeTruthValue ctv = (currentTv.getType() == COMPOSITE_TRUTH_VALUE) ?
                                  CompositeTruthValue((const CompositeTruthValue&) currentTv) :
                                  CompositeTruthValue(currentTv, NULL_VERSION_HANDLE);
        ctv.setVersionedTV(tv, vh);
        TLB::getAtom(h)->setTruthValue(ctv); // always call setTruthValue to update indices
    }
    else
    {
        if (currentTv.getType() == COMPOSITE_TRUTH_VALUE &&
                tv.getType() != COMPOSITE_TRUTH_VALUE)
        {
            CompositeTruthValue ctv((const CompositeTruthValue&) currentTv);
            ctv.setVersionedTV(tv, vh);
            TLB::getAtom(h)->setTruthValue(ctv);
        }
        else
        {
            TLB::getAtom(h)->setTruthValue(tv);
        }
    }
}

const TruthValue& AtomSpaceImpl::getTV(Handle h, VersionHandle vh) const
{
    if (TLB::isInvalidHandle(h)) return TruthValue::NULL_TV();

    const TruthValue& tv  = TLB::getAtom(h)->getTruthValue();
    if (isNullVersionHandle(vh))
     {
        return tv;
    }
    else if (tv.getType() == COMPOSITE_TRUTH_VALUE)
    {
        return ((const CompositeTruthValue&) tv).getVersionedTV(vh);
    }
    return TruthValue::NULL_TV();
}

void AtomSpaceImpl::setMean(Handle h, float mean) throw (InvalidParamException)
{
    TruthValue* newTv = getTV(h).clone();
    if (newTv->getType() == COMPOSITE_TRUTH_VALUE) {
        // Since CompositeTV has no setMean() method, we must handle it differently
        CompositeTruthValue* ctv = (CompositeTruthValue*) newTv;
        TruthValue* primaryTv = ctv->getPrimaryTV().clone();
        if (primaryTv->getType() == SIMPLE_TRUTH_VALUE) {
            ((SimpleTruthValue*)primaryTv)->setMean(mean);
        } else if (primaryTv->getType() == INDEFINITE_TRUTH_VALUE) {
            ((IndefiniteTruthValue*)primaryTv)->setMean(mean);
        } else {
            throw InvalidParamException(TRACE_INFO,
                                        "AtomSpaceImpl - Got a primaryTV with an invalid or unknown type");
        }
        ctv->setVersionedTV(*primaryTv, NULL_VERSION_HANDLE);
        delete primaryTv;
    } else {
        if (newTv->getType() == SIMPLE_TRUTH_VALUE) {
            ((SimpleTruthValue*)newTv)->setMean(mean);
        } else if (newTv->getType() == INDEFINITE_TRUTH_VALUE) {
            ((IndefiniteTruthValue*)newTv)->setMean(mean);
        } else {
            throw InvalidParamException(TRACE_INFO,
                                        "AtomSpaceImpl - Got a TV with an invalid or unknown type");
        }
    }
    setTV(h, *newTv);
    delete newTv;
}


const AttentionValue& AtomSpaceImpl::getAV(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue();
}

void AtomSpaceImpl::setAV(AttentionValueHolder *avh, const AttentionValue& av)
{
    const AttentionValue& oldAV = avh->getAttentionValue();
    // Add the old attention values to the AtomSpace funds and
    // subtract the new attention values from the AtomSpace funds
    fundsSTI += (oldAV.getSTI() - av.getSTI());
    fundsLTI += (oldAV.getLTI() - av.getLTI());

    avh->setAttentionValue(av); // setAttentionValue takes care of updating indices
}

void AtomSpaceImpl::setSTI(AttentionValueHolder *avh, AttentionValue::sti_t stiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(stiValue, currentAv.getLTI(), currentAv.getVLTI()));
}

void AtomSpaceImpl::setLTI(AttentionValueHolder *avh, AttentionValue::lti_t ltiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), ltiValue, currentAv.getVLTI()));
}

void AtomSpaceImpl::setVLTI(AttentionValueHolder *avh, AttentionValue::vlti_t vltiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), currentAv.getLTI(), vltiValue));
}

AttentionValue::sti_t AtomSpaceImpl::getSTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getSTI();
}

float AtomSpaceImpl::getNormalisedSTI(AttentionValueHolder *avh, bool average, bool clip) const
{
    // get normalizer (maxSTI - attention boundary)
    int normaliser;
    float val;
    AttentionValue::sti_t s = getSTI(avh);
    if (s > getAttentionalFocusBoundary()) {
        normaliser = (int) getMaxSTI(average) - getAttentionalFocusBoundary();
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s - getAttentionalFocusBoundary()) / (float) normaliser;
    } else {
        normaliser = -((int) getMinSTI(average) + getAttentionalFocusBoundary());
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s + getAttentionalFocusBoundary()) / (float) normaliser;
    }
    if (clip) {
        return max(-1.0f,min(val,1.0f));
    } else {
        return val;
    }
}

float AtomSpaceImpl::getNormalisedZeroToOneSTI(AttentionValueHolder *avh, bool average, bool clip) const
{
    int normaliser;
    float val;
    AttentionValue::sti_t s = getSTI(avh);
    normaliser = getMaxSTI(average) - getMinSTI(average);
    if (normaliser == 0) {
        return 0.0f;
    }
    val = (s - getMinSTI(average)) / (float) normaliser;
    if (clip) {
        return max(0.0f,min(val,1.0f));
    } else {
        return val;
    }
}

AttentionValue::lti_t AtomSpaceImpl::getLTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getLTI();
}

AttentionValue::vlti_t AtomSpaceImpl::getVLTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getVLTI();
}

float AtomSpaceImpl::getCount(Handle h) const
{
    DPRINTF("AtomSpaceImpl::getCount Atom space address: %p\n", this);
    return TLB::getAtom(h)->getTruthValue().getCount();
}

int AtomSpaceImpl::Nodes(VersionHandle vh) const
{
    DPRINTF("AtomSpaceImpl::Nodes Atom space address: %p\n", this);

    /* This is too expensive and depending on an Agent that may be disabled.
     * Besides it does not have statistics by VersionHandles
     DynamicsStatisticsAgent *agent=DynamicsStatisticsAgent::getInstance();
     agent->reevaluateAllStatistics();
     return agent->getNodeCount();
     */
    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(NODE, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpaceImpl::decayShortTermImportance()
{
    DPRINTF("AtomSpaceImpl::decayShortTermImportance Atom space address: %p\n", this);
    HandleEntry* oldAtoms = atomTable.decayShortTermImportance();
    if (oldAtoms) {
        // Remove from indexes
        atomTable.clearIndexesAndRemoveAtoms(oldAtoms);
        // Send signals
        HandleEntry* current;
        for (current = oldAtoms; current != NULL; current = current->next) {
            // emit remove atom signal
            _removeAtomSignal(this,current->handle);
        }
        // actually remove atoms from TLB
        atomTable.removeExtractedHandles(oldAtoms);
        delete oldAtoms;
    }
}

long AtomSpaceImpl::getTotalSTI() const
{
    long totalSTI = 0;
    HandleEntry* q;

    /* get atom iterator, go through each atom and calculate add
     * sti to total */

    HandleEntry* h = getAtomTable().getHandleSet(ATOM, true);
    q = h;
    while (q) {
        totalSTI += getSTI(q->handle);
        q = q->next;
    }
    delete h;
    return totalSTI;

}

long AtomSpaceImpl::getTotalLTI() const
{
    long totalLTI = 0;
    HandleEntry* q;

    /* get atom iterator, go through each atom and calculate add
     * lti to total */

    HandleEntry* h = getAtomTable().getHandleSet(ATOM, true);
    q = h;
    while (q) {
        totalLTI += getLTI(q->handle);
        q = q->next;
    }
    delete h;
    return totalLTI;

}

long AtomSpaceImpl::getSTIFunds() const
{
    return fundsSTI;
}

long AtomSpaceImpl::getLTIFunds() const
{
    return fundsLTI;
}

int AtomSpaceImpl::Links(VersionHandle vh) const
{
    DPRINTF("AtomSpaceImpl::Links Atom space address: %p\n", this);

    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(LINK, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpaceImpl::_getNextAtomPrepare()
{
    _handle_iterator = atomTable.getHandleIterator(ATOM, true);
}

Handle AtomSpaceImpl::_getNextAtom()
{
    if (_handle_iterator->hasNext())
        return _handle_iterator->next();
    else {
        delete _handle_iterator;
        _handle_iterator = NULL;
        return Handle::UNDEFINED;
    }
}

void AtomSpaceImpl::_getNextAtomPrepare_type(Type type)
{
    type_itr = atomTable.typeIndex.begin(type, true);
}

Handle AtomSpaceImpl::_getNextAtom_type(Type type)
{
    Handle h = *type_itr;
    type_itr ++;
    return h;
}

AttentionValue::sti_t AtomSpaceImpl::getAttentionalFocusBoundary() const
{
    return attentionalFocusBoundary;
}

AttentionValue::sti_t AtomSpaceImpl::setAttentionalFocusBoundary(AttentionValue::sti_t s)
{
    attentionalFocusBoundary = s;
    return s;
}

void AtomSpaceImpl::updateMaxSTI(AttentionValue::sti_t m)
{ maxSTI.update(m); }

AttentionValue::sti_t AtomSpaceImpl::getMaxSTI(bool average) const
{
    if (average) {
        return (AttentionValue::sti_t) maxSTI.recent;
    } else {
        return maxSTI.val;
    }
}

void AtomSpaceImpl::updateMinSTI(AttentionValue::sti_t m)
{ minSTI.update(m); }

AttentionValue::sti_t AtomSpaceImpl::getMinSTI(bool average) const
{
    if (average) {
        return (AttentionValue::sti_t) minSTI.recent;
    } else {
        return minSTI.val;
    }
}

bool AtomSpaceImpl::saveToXML(const std::string& filename) const {
    // Saving to XML is messed up so it always fails for now
    return false;
#if 0
    // This should possible be moved out of the AtomSpace and made into
    // generalised saving interface. This code was moved from
    // SaveRequest because it uses the AtomTable and it'd be (even more)
    // inefficient to make a HandleEntry *and* HandleSeq of all atom handles.

    // This blocks the (planned) atomspace event loop until
    // save is completed.
    pthread_mutex_lock(&atomSpaceLock);

    // XXX/FIXME This is an insanely inefficient way to export the 
    // atomspace! For anything containing a million or more handles,
    // this is just mind-bogglingly bad, as it will results in a vast
    // amount of mallocs & frees, and blow out RAM usage.  Strongly
    // suggest redesign using appropriate iterators.  Anyway, should
    // probably be exporting to scheme, not XML, anyway ... since XML
    // is slow in general.
    HandleEntry *handles = getAtomTable().getHandleSet(ATOM, true);
    NMXmlExporter exporter(this);
    std::fstream file(filename.c_str(), std::fstream::out);
    bool rc = true;
    try {
        file << exporter.toXML(handles);
        rc = true;
    } catch (StandardException &e) {
        logger().error("AtomSpace::saveToXML exception (%s)", e.getMessage());
        rc = false;
    }
    pthread_mutex_unlock(&atomSpaceLock);
    file.flush();
    file.close();
    delete handles;
    return rc;
#endif
}

void AtomSpaceImpl::clear()
{
    std::vector<Handle> allAtoms;
    std::vector<Handle>::iterator i;
    std::back_insert_iterator< std::vector<Handle> > outputI(allAtoms);
    bool result;

    getHandleSet(outputI, ATOM, true);

    int j = 0;
    DPRINTF("%d nodes %d links to erase\n", Nodes(NULL_VERSION_HANDLE),
            Links(NULL_VERSION_HANDLE));
    DPRINTF("atoms in allAtoms: %d\n",allAtoms.size());

    logger().enable();
    logger().setLevel(Logger::DEBUG);

    for (i = allAtoms.begin(); i != allAtoms.end(); i++) {
        result = removeAtom(*i,true);
        if (result) {
            DPRINTF("%d: Atom %u removed, %d nodes %d links left to delete\n",
                j,*i,Nodes(NULL_VERSION_HANDLE), Links(NULL_VERSION_HANDLE));
            j++;
        }
    }

    allAtoms.clear();
    std::back_insert_iterator< std::vector<Handle> > output2(allAtoms);
    getHandleSet(output2, ATOM, true);
    assert(allAtoms.size() == 0);
}

