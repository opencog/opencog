/*
 * opencog/atomspace/AtomSpace.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
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

#include "AtomSpace.h"

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

AtomSpace::AtomSpace(void) : 
    type_itr(0,false)
{
    _handle_iterator = NULL;
    emptyName = "";

    fundsSTI = config().get_int("STARTING_STI_FUNDS");
    fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;

    logger().fine("Max load factor for handle map is: %f", TLB::handle_map.max_load_factor());

}

AtomSpace::~AtomSpace()
{
    // Check if has already been deleted. See in code where it can be delete.
    if (_handle_iterator) {
        delete _handle_iterator;
         _handle_iterator = NULL;
    }
}

Handle AtomSpace::getSpaceMapNode() 
{
    Handle result = getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME);
    if (result == Handle::UNDEFINED) 
    {
        result = addNode(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME);
        setLTI(result, 1);
    } 
    else 
    {
        if (getLTI(result) < 1) 
        {
            setLTI(result, 1);
        }
    }
    return result;
}

AtomSpace& AtomSpace::operator=(const AtomSpace& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomSpace - Cannot copy an object of this class");
}

AtomSpace::AtomSpace(const AtomSpace& other):
    type_itr(0,false)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "AtomSpace - Cannot copy an object of this class");
}

const TruthValue& AtomSpace::getDefaultTV()
{
    return TruthValue::DEFAULT_TV();
}

Type AtomSpace::getAtomType(const string& str) const
{
    DPRINTF("AtomSpace::getAtomType Atom space address: %p\n", this);

    return classserver().getType(const_cast<char*>(str.c_str()));
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

bool AtomSpace::isVar(Handle h) const
{
    DPRINTF("AtomSpace::isVar Atom space address: %p\n", this);
    return inheritsType(getType(h), VARIABLE_NODE);
}

bool AtomSpace::isList(Handle h) const
{
    DPRINTF("AtomSpace::isList Atom space address: %p\n", this);
    return inheritsType(getType(h), LIST_LINK);
}

bool AtomSpace::containsVar(Handle h) const
{
    DPRINTF("AtomSpace::containsVar Atom space address: %p\n", this);

    Node *nnn = dynamic_cast<Node *>(TLB::getAtom(h));
    if (!nnn) {
        for (int i = 0;i < getArity(h);++i)
            if (containsVar(getOutgoing(h, i)))
                return true;
        return false;
    }
    return isVar(h);
}

Handle AtomSpace::createHandle(Type t, const string& str, bool managed)
{
    DPRINTF("AtomSpace::createHandle Atom space address: %p\n", this);

    Handle h = getHandle(t, str);
    return TLB::isValidHandle(h) ? h : addNode(t, str, TruthValue::NULL_TV());
}

Handle AtomSpace::createHandle(Type t, const HandleSeq& outgoing, bool managed)
{
    DPRINTF("AtomSpace::createHandle Atom space address: %p\n", this);

    Handle h = getHandle(t, outgoing);
    return TLB::isValidHandle(h) ? h : addLink(t, outgoing, TruthValue::NULL_TV());
}

bool AtomSpace::containsVersionedTV(Handle h, VersionHandle vh) const
{
    DPRINTF("AtomSpace::containsVersionedTV Atom space address: %p\n", this);

    bool result = isNullVersionHandle(vh);
    if (!result) {
        TruthValuePtr tv(getTV(h));
        result = !tv->isNullTv() && tv->getType() == COMPOSITE_TRUTH_VALUE &&
                 !(boost::shared_dynamic_cast<CompositeTruthValue>(tv)->getVersionedTV(vh).isNullTv());
    }
    return result;
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
    do_merge_tv(result,newTV);
    return result;
}

boost::shared_ptr<Atom> AtomSpace::cloneAtom(const Handle h) const
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

boost::shared_ptr<Node> AtomSpace::cloneNode(const Handle h) const
{
    return boost::shared_dynamic_cast<Node>(this->cloneAtom(h));
}

boost::shared_ptr<Link> AtomSpace::cloneLink(const Handle h) const
{
    return boost::shared_dynamic_cast<Link>(this->cloneAtom(h));
}

size_t AtomSpace::getAtomHash(const Handle h) const 
{
    return TLB::getAtom(h)->hashCode();
}

bool AtomSpace::isValidHandle(const Handle h) const 
{
    return TLB::isValidHandle(h);
}

bool AtomSpace::commitAtom(const Atom& a)
{
    // TODO: Check for differences and abort if timestamp is out of date

    // Get the version in the TLB
    Atom* original = TLB::getAtom(a.getHandle());
    // The only mutable properties of atoms are the TV and AttentionValue
    original->setTruthValue(a.getTruthValue());
    original->setAttentionValue(a.getAttentionValue());
    return true;
}

AttentionValue AtomSpace::getAV(Handle h) const
{
    return atomSpaceAsync.getAV(h)->get_result();
}

void AtomSpace::setAV(Handle h, const AttentionValue &av)
{
    atomSpaceAsync.setAV(h,av)->get_result();
}

const AttentionValue& AtomSpace::getAV(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue();
}

void AtomSpace::setAV(AttentionValueHolder *avh, const AttentionValue& av)
{
    const AttentionValue& oldAV = avh->getAttentionValue();
    // Add the old attention values to the AtomSpace funds and
    // subtract the new attention values from the AtomSpace funds
    fundsSTI += (oldAV.getSTI() - av.getSTI());
    fundsLTI += (oldAV.getLTI() - av.getLTI());

    avh->setAttentionValue(av); // setAttentionValue takes care of updating indices
}

void AtomSpace::setSTI(AttentionValueHolder *avh, AttentionValue::sti_t stiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(stiValue, currentAv.getLTI(), currentAv.getVLTI()));
}

void AtomSpace::setLTI(AttentionValueHolder *avh, AttentionValue::lti_t ltiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), ltiValue, currentAv.getVLTI()));
}

void AtomSpace::setVLTI(AttentionValueHolder *avh, AttentionValue::vlti_t vltiValue)
{
    const AttentionValue& currentAv = getAV(avh);
    setAV(avh, AttentionValue(currentAv.getSTI(), currentAv.getLTI(), vltiValue));
}

AttentionValue::sti_t AtomSpace::getSTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getSTI();
}

float AtomSpace::getNormalisedSTI(AttentionValueHolder *avh, bool average, bool clip) const
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

float AtomSpace::getNormalisedZeroToOneSTI(AttentionValueHolder *avh, bool average, bool clip) const
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

AttentionValue::lti_t AtomSpace::getLTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getLTI();
}

AttentionValue::vlti_t AtomSpace::getVLTI(AttentionValueHolder *avh) const
{
    return avh->getAttentionValue().getVLTI();
}

float AtomSpace::getCount(Handle h) const
{
    DPRINTF("AtomSpace::getCount Atom space address: %p\n", this);
    return TLB::getAtom(h)->getTruthValue().getCount();
}

int AtomSpace::Nodes(VersionHandle vh) const
{
    DPRINTF("AtomSpace::Nodes Atom space address: %p\n", this);

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

void AtomSpace::decayShortTermImportance()
{
    atomSpaceAsync.decayShortTermImportance()->get_result();
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

long AtomSpace::getSTIFunds() const
{
    return fundsSTI;
}

long AtomSpace::getLTIFunds() const
{
    return fundsLTI;
}

int AtomSpace::Links(VersionHandle vh) const
{
    DPRINTF("AtomSpace::Links Atom space address: %p\n", this);

    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(LINK, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpace::_getNextAtomPrepare()
{
    _handle_iterator = atomTable.getHandleIterator(ATOM, true);
}

Handle AtomSpace::_getNextAtom()
{
    if (_handle_iterator->hasNext())
        return _handle_iterator->next();
    else {
        delete _handle_iterator;
        _handle_iterator = NULL;
        return Handle::UNDEFINED;
    }
}

void AtomSpace::_getNextAtomPrepare_type(Type type)
{
    type_itr = atomTable.typeIndex.begin(type, true);
}

Handle AtomSpace::_getNextAtom_type(Type type)
{
    Handle h = *type_itr;
    type_itr ++;
    return h;
}

AttentionValue::sti_t AtomSpace::getAttentionalFocusBoundary() const
{
    return attentionalFocusBoundary;
}

AttentionValue::sti_t AtomSpace::setAttentionalFocusBoundary(AttentionValue::sti_t s)
{
    attentionalFocusBoundary = s;
    return s;
}

void AtomSpace::updateMaxSTI(AttentionValue::sti_t m)
{ maxSTI.update(m); }

AttentionValue::sti_t AtomSpace::getMaxSTI(bool average) const
{
    if (average) {
        return (AttentionValue::sti_t) maxSTI.recent;
    } else {
        return maxSTI.val;
    }
}

void AtomSpace::updateMinSTI(AttentionValue::sti_t m)
{ minSTI.update(m); }

AttentionValue::sti_t AtomSpace::getMinSTI(bool average) const
{
    if (average) {
        return (AttentionValue::sti_t) minSTI.recent;
    } else {
        return minSTI.val;
    }
}

void AtomSpace::clear()
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

