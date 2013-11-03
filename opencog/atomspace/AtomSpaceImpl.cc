/*
 * opencog/atomspace/AtomSpaceImpl.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
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

#include "AtomSpaceImpl.h"

#include <string>
#include <iostream>
#include <fstream>
#include <list>

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

AtomSpaceImpl::AtomSpaceImpl(void)
    : bank(&atomTable)
{
    backing_store = NULL;

    // connect signals
    addedAtomConnection = addAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomAdded, this, _1));
    removedAtomConnection = atomTable.removeAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomRemoved, this, _1));

    DPRINTF("AtomSpaceImpl::Constructor AtomTable address: %p\n", &atomTable);
}

AtomSpaceImpl::~AtomSpaceImpl()
{
    // disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();
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

void AtomSpaceImpl::atomAdded(Handle h)
{
    DPRINTF("AtomSpaceImpl::atomAdded(%lu): %s\n", h.value(), h->toShortString().c_str());
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

void AtomSpaceImpl::atomRemoved(AtomPtr atom)
{
    Type type = atom->getType();
    if (type == CONTEXT_LINK) {
        // Remove corresponding VersionedTV to the contextualized atom
        // Note that when a VersionedTV is removed from a
        // CompositeTruthValue it will not automatically remove the
        // corresponding ContextLink
        LinkPtr lll(LinkCast(atom));
        OC_ASSERT(lll->getArity() == 2,
            "AtomSpaceImpl::atomRemoved: Got invalid arity for removed ContextLink = %d\n",
            lll->getArity());
        Handle cx = lll->getOutgoingAtom(0); // context
        Handle ca = lll->getOutgoingAtom(1); // contextualized atom
        TruthValuePtr tv = getTV(ca);
        CompositeTruthValuePtr new_ctv(CompositeTruthValue::createCTV(tv));
        new_ctv->removeVersionedTV(VersionHandle(CONTEXTUAL, cx));
        // @todo: one may want improve that code by converting back
        // the CompositeTV into a simple or indefinite TV when it has
        // no more VersionedTV
        setTV(ca, new_ctv);
    }
}

// ====================================================================

void AtomSpaceImpl::print(std::ostream& output, Type type, bool subclass) const
{
    atomTable.print(output, type, subclass);
}

AtomSpaceImpl& AtomSpaceImpl::operator=(const AtomSpaceImpl& other)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpaceImpl - Cannot copy an object of this class");
}

AtomSpaceImpl::AtomSpaceImpl(const AtomSpaceImpl& other)
    : bank(&atomTable)
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpaceImpl - Cannot copy an object of this class");
}

Handle AtomSpaceImpl::addNode(Type t, const string& name, TruthValuePtr tvn)
{
    DPRINTF("AtomSpaceImpl::addNode AtomTable address: %p\n", &atomTable);
    DPRINTF("====AtomTable.linkIndex address: %p size: %d\n", &atomTable.linkIndex, atomTable.linkIndex.idx.size());

    // Maybe the backing store knows about this atom.
// XXX this is utterly the wrong place to do this ... 
    if (backing_store) {
        NodePtr n(backing_store->getNode(t, name.c_str()));
        if (n) {
            Handle result = atomTable.add(n);
            atomTable.merge(result,tvn);
            return result;
        }
    }

    NodePtr n(createNode(t, name, tvn));
    Handle newNodeHandle = atomTable.add(n);
    return newNodeHandle;
}

Handle AtomSpaceImpl::addLink(Type t, const HandleSeq& outgoing,
                              TruthValuePtr tvn)
{
    DPRINTF("AtomSpaceImpl::addLink AtomTable address: %p\n", &atomTable);
    DPRINTF("====AtomTable.linkIndex address: %p size: %d\n", &atomTable.linkIndex, atomTable.linkIndex.idx.size());

// XXX this is utterly the wrong place to do this ... 
    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        LinkPtr l(backing_store->getLink(t, outgoing));
        if (l) {
            // register the atom with the atomtable (so it gets placed in
            // indices)
            Handle result = atomTable.add(l);
            atomTable.merge(result,tvn);
            return result;
        }
    }

    Handle newLinkHandle = atomTable.add(createLink(t, outgoing, tvn));
    return newLinkHandle;
}

Handle AtomSpaceImpl::fetchAtom(Handle h)
{
    // No-op if we've already got this handle.
    // XXX But perhaps we want to update the truth value from the
    // remote storage?? The semantics of this are totally unclear.
    if (atomTable.holds(h)) return h;

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        AtomPtr a(backing_store->getAtom(h));

        // For links, must perform a recursive fetch, as otherwise
        // the atomtable.add below will throw an error.
        LinkPtr l(LinkCast(a));
        if (l) {
           const HandleSeq& ogs = l->getOutgoingSet();
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
    if (backing_store) {
        HandleSeq iset = backing_store->getIncomingSet(h);
        size_t isz = iset.size();
        for (size_t i=0; i<isz; i++) {
            Handle hi = iset[i];
            if (recursive) {
                fetchIncomingSet(hi, true);
            } else {
                fetchAtom(hi);
            }
        }
    }
    return base;
}

HandleSeq AtomSpaceImpl::getNeighbors(Handle h, bool fanin,
        bool fanout, Type desiredLinkType, bool subClasses) const
{
    if (h == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Handle %d doesn't refer to a Atom", h.value());
    }
    HandleSeq answer;

    const UnorderedHandleSet& iset = atomTable.getIncomingSet(h);
    for (UnorderedHandleSet::const_iterator it = iset.begin();
         it != iset.end(); it++)
    {
        LinkPtr link(LinkCast(*it));
        Type linkType = link->getType();
        DPRINTF("Atom::getNeighbors(): linkType = %d desiredLinkType = %d\n", linkType, desiredLinkType);
        if ((linkType == desiredLinkType) || (subClasses && classserver().isA(linkType, desiredLinkType))) {
            Arity linkArity = link->getArity();
            for (Arity i = 0; i < linkArity; i++) {
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

    const UnorderedHandleSet& iset = atomTable.getIncomingSet(h);
    HandleSeq hs;
    std::copy(iset.begin(), iset.end(), back_inserter(hs));
    return hs;
}

bool AtomSpaceImpl::setTV(Handle h, TruthValuePtr tv, VersionHandle vh)
{
    // Due to the way AsyncRequest is designed, we are being called
    // with a null atom pointer; only the uuid is valid. So first,
    // go fetch the actual atom out of the table.
    h = atomTable.getHandle(h);

    if (!h) return false;
    TruthValuePtr currentTv = h->getTruthValue();
    if (!isNullVersionHandle(vh))
    {
        CompositeTruthValuePtr ctv = (currentTv->getType() == COMPOSITE_TRUTH_VALUE) ?
                            CompositeTruthValue::createCTV(currentTv) :
                            CompositeTruthValue::createCTV(currentTv, NULL_VERSION_HANDLE);
        ctv->setVersionedTV(tv, vh);
        h->setTruthValue(ctv); // always call setTruthValue to update indices
    } else {
        if (currentTv->getType() == COMPOSITE_TRUTH_VALUE &&
                tv->getType() != COMPOSITE_TRUTH_VALUE)
        {
            CompositeTruthValuePtr ctv(CompositeTruthValue::createCTV(currentTv));
            ctv->setVersionedTV(tv, vh);
            h->setTruthValue(ctv);
        } else {
            h->setTruthValue(tv);
        }
    }

    return true;
}

TruthValuePtr AtomSpaceImpl::getTV(Handle h, VersionHandle vh) const
{
    // Due to the way AsyncRequest is designed, we are being called
    // with a null atom pointer; only the uuid is valid. So first,
    // go fetch the actual atom out of the table.
    h = atomTable.getHandle(h);
    if (!h) return TruthValue::NULL_TV();

    TruthValuePtr tv = h->getTruthValue();
    if (isNullVersionHandle(vh)) {
        return tv;
    }
    else if (tv->getType() == COMPOSITE_TRUTH_VALUE)
    {
        return std::dynamic_pointer_cast<CompositeTruthValue>(tv)->getVersionedTV(vh);
    }
    return TruthValue::NULL_TV();
}

void AtomSpaceImpl::setMean(Handle h, float mean) throw (InvalidParamException)
{
    TruthValuePtr newTv = getTV(h);
    if (newTv->getType() == COMPOSITE_TRUTH_VALUE) {
        // Since CompositeTV has no setMean() method, we must handle it differently
        CompositeTruthValuePtr ctv(CompositeTruthValue::createCTV(newTv));

        TruthValuePtr primaryTv = ctv->getPrimaryTV();
        if (primaryTv->getType() == SIMPLE_TRUTH_VALUE) {
            primaryTv = SimpleTruthValue::createTV(mean, primaryTv->getCount());
        } else if (primaryTv->getType() == INDEFINITE_TRUTH_VALUE) {
            IndefiniteTruthValuePtr itv = IndefiniteTruthValue::createITV(primaryTv);
            itv->setMean(mean);
            primaryTv = itv;
        } else {
            throw InvalidParamException(TRACE_INFO,
               "AtomSpaceImpl - Got a primaryTV with an invalid or unknown type");
        }
        ctv->setVersionedTV(primaryTv, NULL_VERSION_HANDLE);
    } else {
        if (newTv->getType() == SIMPLE_TRUTH_VALUE) {
            newTv = SimpleTruthValue::createTV(mean, newTv->getCount());
        } else if (newTv->getType() == INDEFINITE_TRUTH_VALUE) {
            IndefiniteTruthValuePtr itv = IndefiniteTruthValue::createITV(newTv);
            itv->setMean(mean);
            newTv = itv;
        } else {
            throw InvalidParamException(TRACE_INFO,
               "AtomSpaceImpl - Got a TV with an invalid or unknown type");
        }
    }
    setTV(h, newTv);
}

float AtomSpaceImpl::getNormalisedSTI(AttentionValuePtr av, bool average, bool clip) const
{
    // get normalizer (maxSTI - attention boundary)
    int normaliser;
    float val;
    AttentionValue::sti_t s = av->getSTI();
    if (s > bank.getAttentionalFocusBoundary()) {
        normaliser = (int) bank.getMaxSTI(average) - bank.getAttentionalFocusBoundary();
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s - bank.getAttentionalFocusBoundary()) / (float) normaliser;
    } else {
        normaliser = -((int) bank.getMinSTI(average) + bank.getAttentionalFocusBoundary());
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s + bank.getAttentionalFocusBoundary()) / (float) normaliser;
    }
    if (clip) {
        return max(-1.0f,min(val,1.0f));
    } else {
        return val;
    }
}

float AtomSpaceImpl::getNormalisedZeroToOneSTI(AttentionValuePtr av, bool average, bool clip) const
{
    int normaliser;
    float val;
    AttentionValue::sti_t s = av->getSTI();
    normaliser = bank.getMaxSTI(average) - bank.getMinSTI(average);
    if (normaliser == 0) {
        return 0.0f;
    }
    val = (s - bank.getMinSTI(average)) / (float) normaliser;
    if (clip) {
        return max(0.0f,min(val,1.0f));
    } else {
        return val;
    }
}

void AtomSpaceImpl::clear()
{
    std::vector<Handle> allAtoms;

    getHandleSet(back_inserter(allAtoms), ATOM, true);

    DPRINTF("%d nodes %d links to erase\n", Nodes(NULL_VERSION_HANDLE),
            Links(NULL_VERSION_HANDLE));
    DPRINTF("atoms in allAtoms: %lu\n",allAtoms.size());

    Logger::Level save = logger().getLevel();
    logger().setLevel(Logger::DEBUG);

    size_t j = 0;
    std::vector<Handle>::iterator i;
    for (i = allAtoms.begin(); i != allAtoms.end(); ++i) {
        bool result = removeAtom(*i, true);
        if (result) {
            DPRINTF("%d: Atom %lu removed, %d nodes %d links left to delete\n",
                j,i->value(),Nodes(NULL_VERSION_HANDLE), Links(NULL_VERSION_HANDLE));
            j++;
        }
    }

    allAtoms.clear();
    getHandleSet(back_inserter(allAtoms), ATOM, true);
    assert(allAtoms.size() == 0);

    logger().setLevel(save);
}

void AtomSpaceImpl::printGDB() const { print(); }
void AtomSpaceImpl::printTypeGDB(Type t) const { print(std::cout,t,true); }
