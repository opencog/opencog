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

AtomSpaceImpl::AtomSpaceImpl(void) : 
    type_itr(0,false)
{
    _handle_iterator = NULL;
    emptyName = "";
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
}

// ====================================================================

const AtomTable& AtomSpaceImpl::getAtomTable() const
{
    DPRINTF("AtomSpaceImpl::getAtomTable this(%p), atomTable address = %p\n", this, &atomTable);
    return atomTable;
}

void AtomSpaceImpl::print(std::ostream& output, Type type, bool subclass) const
{
    atomTable.print(output, type, subclass);
}

Handle AtomSpaceImpl::getHandle(Type t, const std::string& str) const {
    return atomTable.getHandle(str.c_str(), t);
}

Handle AtomSpaceImpl::getHandle(Type t, const HandleSeq& outgoing) const {
    return atomTable.getHandle(t, outgoing);
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

Type AtomSpaceImpl::getType(Handle h) const
{
    DPRINTF("AtomSpaceImpl::getType Atom space address: %p\n", this);
    Atom* a = atomTable.getAtom(h);
    if (a) return a->getType();
    else return NOTYPE;
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

AttentionBank& AtomSpaceImpl::getAttentionBank() { return bank; }

bool AtomSpaceImpl::removeAtom(Handle h, bool recursive)
{
    HandleEntry* extractedHandles = atomTable.extract(h, recursive);
    if (extractedHandles) {
        HandleEntry* currentEntry = extractedHandles;
        while (currentEntry) {
            Handle h = currentEntry->handle;

            // Also refund sti/lti to AtomSpace funds pool
            bank.updateSTIFunds(getSTI(h));
            bank.updateLTIFunds(getLTI(h));

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
    Link *link = atomTable.getLink(h);
    if (!link) return hs;
    return link->getOutgoingSet();
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
        Node *n = backing_store->getNode(t, name.c_str());
        if (n) {
            result = atomTable.add(n);
            atomTable.merge(result,tvn);
            // emit "merge atom" signal
            _mergeAtomSignal(this,result);
            return result;
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
        Link *l = backing_store->getLink(t, outgoing);
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

    Handle newLinkHandle = atomTable.add(new Link(t, outgoing, tvn));
    // emit add atom signal
    _addAtomSignal(this,newLinkHandle);
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
        Atom *a = backing_store->getAtom(h);

        // For links, must perform a recursive fetch, as otherwise
        // the atomtable.add below will throw an error.
        Link *l = dynamic_cast<Link *>(a);
        if (l) {
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
    if (backing_store) {
        std::vector<Handle> iset = backing_store->getIncomingSet(h);
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
    Node * nnn = atomTable.getNode(h);
    if (nnn)
        return nnn->getName();
    else
        return emptyName;
}

boost::shared_ptr<Atom> AtomSpaceImpl::cloneAtom(const Handle& h) const
{
    // TODO: Add timestamp to atoms and add vector clock to AtomSpace
    // Need to use the newly added clone methods as the copy constructors for
    // Node and Link don't copy incoming set.
    Atom * a = atomTable.getAtom(h);
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

std::string AtomSpaceImpl::atomAsString(Handle h, bool terse) const
{
    Atom* a = atomTable.getAtom(h);
    if (a) {
        if (terse) return a->toShortString();
        else return a->toString();
    }
    return std::string("ERROR: Bad handle");
}

HandleSeq AtomSpaceImpl::getNeighbors(const Handle& h, bool fanin,
        bool fanout, Type desiredLinkType, bool subClasses) const 
{
    Atom* a = atomTable.getAtom(h);
    if (a == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Handle %d doesn't refer to a Atom", h.value());
    }
    HandleSeq answer;

    for (HandleEntry *he = atomTable.getIncomingSet(h); he != NULL; he = he ->next) {
        Link *link = atomTable.getLink(he->handle);
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
    const Link *l = atomTable.getLink(link);
    if (l != NULL) {
        return l->isSource(source);
    }
    return false;
}

size_t AtomSpaceImpl::getAtomHash(const Handle& h) const 
{
    return atomTable.getAtom(h)->hashCode();
}

bool AtomSpaceImpl::isValidHandle(const Handle& h) const 
{
    return atomTable.holds(h);
}

bool AtomSpaceImpl::commitAtom(const Atom& a)
{
    // TODO: Check for differences and abort if timestamp is out of date

    // Get the version in the atomTable
    Atom* original = atomTable.getAtom(a.getHandle());
    if (original == NULL)
        // TODO: allow committing a new atom?
        return false;
    // The only mutable properties of atoms are the TV and AttentionValue
    // TODO: this isn't correct, trails, flags and other things might change
    // too...
    original->setTruthValue(a.getTruthValue());
    original->setAttentionValue(a.getAttentionValue());
    return true;
}

Handle AtomSpaceImpl::getOutgoing(Handle h, int idx) const
{
    Link * l = atomTable.getLink(h);
    if (NULL == l)
        throw new IndexErrorException(TRACE_INFO,
            "Asked for outgoing set on atom that is not a link!\n");
    if (idx >= l->getArity())
        throw new IndexErrorException(TRACE_INFO, "Invalid outgoing set index: %d (atom: %s)\n",
                                      idx, l->toString().c_str());
    return l->getOutgoingSet()[idx];
}

int AtomSpaceImpl::getArity(Handle h) const
{
    Link * l = atomTable.getLink(h);
    if (NULL == l) return 0;
    return l->getArity();
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

    HandleEntry* he = atomTable.getIncomingSet(h);
    if (he) return he->toHandleVector();
    else return HandleSeq();
}

bool AtomSpaceImpl::setTV(Handle h, const TruthValue& tv, VersionHandle vh)
{
    Atom *a = atomTable.getAtom(h);
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
    Atom* a = atomTable.getAtom(h);
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

float AtomSpaceImpl::getNormalisedSTI(AttentionValueHolder *avh, bool average, bool clip) const
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

float AtomSpaceImpl::getNormalisedZeroToOneSTI(AttentionValueHolder *avh, bool average, bool clip) const
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
        // actually remove atoms from AtomTable
        atomTable.removeExtractedHandles(oldAtoms);
        delete oldAtoms;
    }
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
    type_itr++;
    return h;
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
    DPRINTF("atoms in allAtoms: %lu\n",allAtoms.size());

    Logger::Level save = logger().getLevel();
    logger().setLevel(Logger::DEBUG);

    for (i = allAtoms.begin(); i != allAtoms.end(); ++i) {
        result = removeAtom(*i,true);
        if (result) {
            DPRINTF("%d: Atom %lu removed, %d nodes %d links left to delete\n",
                j,i->value(),Nodes(NULL_VERSION_HANDLE), Links(NULL_VERSION_HANDLE));
            j++;
        }
    }

    allAtoms.clear();
    std::back_insert_iterator< std::vector<Handle> > output2(allAtoms);
    getHandleSet(output2, ATOM, true);
    assert(allAtoms.size() == 0);

    logger().setLevel(save);
}

void AtomSpaceImpl::printGDB() const { print(); }
void AtomSpaceImpl::printTypeGDB(Type t) const { print(std::cout,t,true); }
