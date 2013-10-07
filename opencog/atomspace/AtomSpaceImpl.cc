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

AtomSpaceImpl::AtomSpaceImpl(void)
{
    backing_store = NULL;

    // connect signals
    addedAtomConnection = addAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomAdded, this, _1, _2));
    removedAtomConnection = removeAtomSignal().connect(boost::bind(&AtomSpaceImpl::atomRemoved, this, _1, _2));

    pthread_mutex_init(&atomSpaceLock, NULL);
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

void AtomSpaceImpl::atomAdded(AtomSpaceImpl *a, Handle h)
{
    DPRINTF("AtomSpaceImpl::atomAdded(%lu): %s\n", h.value(), atomAsString(h).c_str());
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

void AtomSpaceImpl::atomRemoved(AtomSpaceImpl *a, AtomPtr atom)
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
        Handle cx = lll->getOutgoingHandle(0); // context
        Handle ca = lll->getOutgoingHandle(1); // contextualized atom
        const TruthValue& tv = getTV(ca);
        OC_ASSERT(tv.getType() == COMPOSITE_TRUTH_VALUE);
        CompositeTruthValue new_ctv(static_cast<const CompositeTruthValue&>(tv));
        new_ctv.removeVersionedTV(VersionHandle(CONTEXTUAL, cx));
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
{
    throw opencog::RuntimeException(TRACE_INFO,
            "AtomSpaceImpl - Cannot copy an object of this class");
}

bool AtomSpaceImpl::removeAtom(Handle h, bool recursive)
{
    AtomPtrSet extractedAtoms = atomTable.extract(h, recursive);
    if (extractedAtoms.size() == 0) return false;

    AtomPtrSet::const_iterator it;
    for (it = extractedAtoms.begin(); it != extractedAtoms.end(); it++) {
        AtomPtr a = *it;

        // Also refund sti/lti to AtomSpace funds pool
        bank.updateSTIFunds(bank.getSTI(a));
        bank.updateLTIFunds(bank.getLTI(a));

        // emit remove atom signal
        _removeAtomSignal(this, a);
    }

    return true;
}

Handle AtomSpaceImpl::addNode(Type t, const string& name, const TruthValue& tvn)
{
    DPRINTF("AtomSpaceImpl::addNode AtomTable address: %p\n", &atomTable);
    DPRINTF("====AtomTable.linkIndex address: %p size: %d\n", &atomTable.linkIndex, atomTable.linkIndex.idx.size());
    Handle result = getHandle(t, name);
    if (atomTable.holds(result)) {
        atomTable.merge(result, tvn);
        // emit "merge atom" signal
        _mergeAtomSignal(this,result);
        return result;
    }

    // Remove default STI/LTI from AtomSpace Funds
    fundsSTI -= AttentionValue::DEFAULTATOMSTI;
    fundsLTI -= AttentionValue::DEFAULTATOMLTI;

    // Maybe the backing store knows about this atom.
    if (backing_store) {
        NodePtr n(backing_store->getNode(t, name.c_str()));
        if (n) {
            result = atomTable.add(n);
            atomTable.merge(result,tvn);
            // emit "merge atom" signal
            _mergeAtomSignal(this,result);
            return result;
        }
    }

    NodePtr n(createNode(t, name, tvn));
    Handle newNodeHandle = atomTable.add(n);
    // emit add atom signal
    _addAtomSignal(this, newNodeHandle);

    return newNodeHandle;
}

Handle AtomSpaceImpl::addLink(Type t, const HandleSeq& outgoing,
                          const TruthValue& tvn)
{
    DPRINTF("AtomSpaceImpl::addLink AtomTable address: %p\n", &atomTable);
    DPRINTF("====AtomTable.linkIndex address: %p size: %d\n", &atomTable.linkIndex, atomTable.linkIndex.idx.size());
    Handle result = getHandle(t, outgoing);
    if (atomTable.holds(result)) {
        // If the node already exists, it must be merged properly
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
        LinkPtr l(backing_store->getLink(t, outgoing));
        if (l) {
            // register the atom with the atomtable (so it gets placed in
            // indices)
            result = atomTable.add(l);
            atomTable.merge(result,tvn);
            // Send a merge signal
            _mergeAtomSignal(this,result);
            return result;
        }
    }

    Handle newLinkHandle = atomTable.add(createLink(t, outgoing, tvn));
    // emit add atom signal
    _addAtomSignal(this, newLinkHandle);
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

AtomPtr AtomSpaceImpl::cloneAtom(Handle h) const
{
    // TODO: Add timestamp to atoms and add vector clock to AtomSpace
    return atomTable.getAtom(h);
}

std::string AtomSpaceImpl::atomAsString(Handle h, bool terse) const
{
    AtomPtr a(atomTable.getAtom(h));
    if (a) {
        if (terse) return a->toShortString();
        else return a->toString();
    }
    return std::string("ERROR: Bad handle");
}

HandleSeq AtomSpaceImpl::getNeighbors(Handle h, bool fanin,
        bool fanout, Type desiredLinkType, bool subClasses) const
{
    AtomPtr a(atomTable.getAtom(h));
    if (a == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Handle %d doesn't refer to a Atom", h.value());
    }
    HandleSeq answer;

    const UnorderedHandleSet& iset = atomTable.getIncomingSet(h);
    for (UnorderedHandleSet::const_iterator it = iset.begin();
         it != iset.end(); it++)
    {
        LinkPtr link(atomTable.getLink(*it));
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

bool AtomSpaceImpl::commitAtom(AtomPtr a)
{
    // TODO: Check for differences and abort if timestamp is out of date

    Handle h = atomTable.getHandle(a);
    AtomPtr original(atomTable.getAtom(h));
    if (original == NULL)
        // TODO: allow committing a new atom?
        return false;
    // The only mutable properties of atoms are the TV and AttentionValue
    // TODO: this isn't correct, trails, flags and other things might change
    // too... XXX the AtomTable already has a merge function; shouldn't we
    // be using that?
    original->setTruthValue(a->getTruthValue());
    original->setAttentionValue(a->getAttentionValue());
    return true;
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

bool AtomSpaceImpl::setTV(Handle h, const TruthValue& tv, VersionHandle vh)
{
    AtomPtr a(atomTable.getAtom(h));
    if (!a) return false;
    const TruthValue& currentTv = a->getTruthValue();
    if (!isNullVersionHandle(vh))
    {
        CompositeTruthValue ctv = (currentTv.getType() == COMPOSITE_TRUTH_VALUE) ?
                                  CompositeTruthValue((const CompositeTruthValue&) currentTv) :
                                  CompositeTruthValue(currentTv, NULL_VERSION_HANDLE);
        ctv.setVersionedTV(tv, vh);
        a->setTruthValue(ctv); // always call setTruthValue to update indices
    } else {
        if (currentTv.getType() == COMPOSITE_TRUTH_VALUE &&
                tv.getType() != COMPOSITE_TRUTH_VALUE) {
            CompositeTruthValue ctv((const CompositeTruthValue&) currentTv);
            ctv.setVersionedTV(tv, vh);
            a->setTruthValue(ctv);
        } else {
            a->setTruthValue(tv);
        }
    }

    return true;
}

const TruthValue& AtomSpaceImpl::getTV(Handle h, VersionHandle vh) const
{
    AtomPtr a(atomTable.getAtom(h));
    if (!a) return TruthValue::NULL_TV();

    const TruthValue& tv = a->getTruthValue();
    if (isNullVersionHandle(vh)) {
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

float AtomSpaceImpl::getNormalisedSTI(AttentionValueHolderPtr avh, bool average, bool clip) const
{
    // get normalizer (maxSTI - attention boundary)
    int normaliser;
    float val;
    AttentionValue::sti_t s = bank.getSTI(avh);
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

float AtomSpaceImpl::getNormalisedZeroToOneSTI(AttentionValueHolderPtr avh, bool average, bool clip) const
{
    int normaliser;
    float val;
    AttentionValue::sti_t s = bank.getSTI(avh);
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

size_t AtomSpaceImpl::Nodes(VersionHandle vh) const
{
    DPRINTF("AtomSpaceImpl::Nodes Atom space address: %p\n", this);

    // The following implementation is still expensive, but already deals with VersionHandles:
    // It would be cheaper if instead we just used foreachHandleByTypeVH
    // and just had a function that simply counts!!
    HandleSeq hs;
    atomTable.getHandlesByTypeVH(back_inserter(hs), NODE, true, vh);
    return hs.size();
}

void AtomSpaceImpl::decayShortTermImportance()
{
    DPRINTF("AtomSpaceImpl::decayShortTermImportance Atom space address: %p\n", this);
    AtomPtrSet oldAtoms = atomTable.decayShortTermImportance();

    // Send signals  -- emit remove atom signal
    AtomPtrSet::const_iterator it;
    AtomPtrSet::const_iterator end = oldAtoms.end();
    for (it = oldAtoms.begin(); it != end; it++)
        _removeAtomSignal(this, (*it));
}


size_t AtomSpaceImpl::Links(VersionHandle vh) const
{
    DPRINTF("AtomSpaceImpl::Links Atom space address: %p\n", this);

    // The following implementation is still expensive, but already deals with VersionHandles:
    // It would be cheaper if instead we just used foreachHandleByTypeVH
    // and just had a function that simply counts!!
    HandleSeq hs;
    atomTable.getHandlesByTypeVH(back_inserter(hs), LINK, true, vh);
    return hs.size();
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
