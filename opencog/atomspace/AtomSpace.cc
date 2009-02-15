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
#include <list>

#include <stdlib.h>
#include <tr1/functional>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/HandleEntry.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

using std::string;
using std::cerr;
using std::cout;
using std::endl;
using namespace std::tr1::placeholders;
using namespace opencog;

// ====================================================================

AtomSpace::AtomSpace(void)
{
    _handle_iterator = NULL;
    emptyName = "";
    backing_store = NULL;

    fundsSTI = config().get_int("STARTING_STI_FUNDS");
    fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;

    //connect signals
    atomTable.addAtomSignal().connect(std::tr1::bind(&AtomSpace::atomAdded, this, _1));
    atomTable.removeAtomSignal().connect(std::tr1::bind(&AtomSpace::atomRemoved, this, _1));
}

AtomSpace::~AtomSpace()
{
    // Check if has already been deleted. See in code where it can be delete.
    if (_handle_iterator) {
        delete (_handle_iterator);
    }
    //delete (_handle_entry);
}

// ====================================================================

void AtomSpace::registerBackingStore(BackingStore *bs)
{
	backing_store = bs;
}

void AtomSpace::unregisterBackingStore(BackingStore *bs)
{
	if (bs == backing_store) backing_store = NULL;
}

// ====================================================================

void AtomSpace::atomAdded(Handle h)
{
    //logger().debug("AtomSpace::atomAdded(%p): %s", h, TLB::getAtom(h)->toString().c_str());
    if (getType(h) == AT_TIME_LINK) {
        // Add corresponding TimeServer entry
        if (getArity(h) == 2) {
            Handle timeNode = getOutgoing(h, 0);
            if (getType(timeNode) == TIME_NODE) {
                const string& timeNodeName = getName(timeNode);
                Temporal t = Temporal::getFromTimeNodeName(timeNodeName.c_str());
                Handle timed_h = getOutgoing(h, 1);
                timeServer.add(timed_h, t);
            } else logger().warn("AtomSpace::addLink: Invalid atom type at the first element in an AtTimeLink's outgoing: %s\n", ClassServer::getTypeName(getType(timeNode)).c_str());
        } else logger().warn("AtomSpace::addLink: Invalid arity for an AtTimeLink: %d (expected: 2)\n", getArity(h));
    }
}

void AtomSpace::atomRemoved(Handle h)
{
    //logger().debug("AtomSpace::atomAdded(%p): %s", h, TLB::getAtom(h)->toString().c_str());
    Type type = getType(h);
    if (type == AT_TIME_LINK) {
        cassert(TRACE_INFO, getArity(h) == 2, "AtomSpace::decayShortTermImportance(): Got invalid arity for removed AtTimeLink = %d\n", getArity(h));
        Handle timeNode = getOutgoing(h, 0);
        cassert(TRACE_INFO, getType(timeNode) == TIME_NODE, "AtomSpace::removeAtom: Got no TimeNode node at the first position of the AtTimeLink\n");
        Handle timedAtom = getOutgoing(h, 1);
        timeServer.remove(timedAtom, Temporal::getFromTimeNodeName(((Node*) TLB::getAtom(timeNode))->getName().c_str()));
    }
}

// ====================================================================

const AtomTable& AtomSpace::getAtomTable() const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
    return atomTable;
}

const TimeServer& AtomSpace::getTimeServer() const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return timeServer;
}


void AtomSpace::print(std::ostream& output, Type type, bool subclass) const
{
    atomTable.print(output, type, subclass);
}

Handle AtomSpace::addTimeInfo(Handle h, unsigned long timestamp, const TruthValue& tv)
{
    cassert(TRACE_INFO, TLB::isValidHandle(h), "AtomSpace::addTimeInfo: Got an invalid handle as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const Temporal& t, const TruthValue& tv)
{
    cassert(TRACE_INFO, TLB::isValidHandle(h), "AtomSpace::addTimeInfo: Got an invalid handle as argument\n");
    cassert(TRACE_INFO, t != UNDEFINED_TEMPORAL, "AtomSpace::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    return addTimeInfo(h, t.getTimeNodeName(), tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv)
{
//    logger().debug("AtomSpace::addTimeInfo - temp init");
    Handle timeNode = addNode(TIME_NODE, timeNodeName.c_str());
//    logger().debug("AtomSpace::addTimeInfo - temp 1");
    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    atTimeLinkOutgoing.push_back(h);
    Handle atTimeLink = addLink(AT_TIME_LINK, atTimeLinkOutgoing, tv);
//    logger().debug("AtomSpace::addTimeInfo - temp end");
    return atTimeLink;
}

bool AtomSpace::removeTimeInfo(Handle h, unsigned long timestamp, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive)
{
    Temporal t(timestamp);
    return removeTimeInfo(h, t, criterion, removeDisconnectedTimeNodes, recursive);
}

bool AtomSpace::removeTimeInfo(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive)
{
    //printf("AtomSpace::removeTimeInfo(%s, %s, %d, %s, %d, %d)\n", TLB::getHandle(h)->toString().c_str(), t.toString().c_str(), TemporalTable::getTemporalRelationshipStr(criterion), removeDisconnectedTimeNodes, recursive);

    std::list<HandleTemporalPair> existingEntries;
    timeServer.get(back_inserter(existingEntries), h, t, criterion);
    bool result = !existingEntries.empty();
    for (std::list<HandleTemporalPair>::const_iterator itr = existingEntries.begin();
            itr != existingEntries.end(); itr++) {
        Handle atTimeLink = getAtTimeLink(*itr);
        //printf("Got atTimeLink = %p\n", atTimeLink);
        if (TLB::isValidHandle(atTimeLink)) {
            Handle timeNode = getOutgoing(atTimeLink, 0);
            //printf("Got timeNode = %p\n", timeNode);
            cassert(TRACE_INFO, getArity(atTimeLink) == 2, "AtomSpace::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n", getArity(atTimeLink));
            cassert(TRACE_INFO, TLB::isValidHandle(timeNode) && getType(timeNode) == TIME_NODE, "AtomSpace::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
            if (removeAtom(atTimeLink, recursive)) {
                //printf("atTimeLink removed from AT successfully\n");
                if (removeDisconnectedTimeNodes && getIncoming(timeNode).empty()) {
                    //printf("Trying to remove timeNode as well\n");
                    removeAtom(timeNode);
                }
            } else {
                result = false;
            }
        } else {
            result = false;
        }
    }
    return result;
}

Handle AtomSpace::getAtTimeLink(const HandleTemporalPair& htp) const
{
    Handle result = Handle::UNDEFINED;

    const Temporal& t = *(htp.getTemporal());
    Handle h = htp.getHandle();
    //printf("AtomSpace::getAtTimeLink: t = %s, h = %s\n", t.toString().c_str(), TLB::getAtom(h)->toString().c_str());

    Handle timeNode = getHandle(TIME_NODE, t.getTimeNodeName().c_str());
    //printf("timeNode = %p\n", timeNode);
    if (TLB::isValidHandle(timeNode)) {
        HandleSeq atTimeLinkOutgoing(2);
        atTimeLinkOutgoing[0] = timeNode;
        atTimeLinkOutgoing[1] = h;
        HandleSeq atTimeLinks;
        getHandleSet(back_inserter(atTimeLinks), atTimeLinkOutgoing, NULL, NULL, 2, AT_TIME_LINK, false);
        if (!atTimeLinks.empty()) {
            result = atTimeLinks[0];
            if (atTimeLinks.size() > 1) {
                logger().warn(
                    "AtomSpace::getAtTimeLink: More than 1 AtTimeLink(TimeNode, TimedAtom) found for HandleTemporalPair = %s \n",
                    htp.toString().c_str());
            }
            //} else {
            //    logger().debug(
            //        "AtomSpace::getAtTimeLink: No corresponding AtTimeLink(TimeNode, TimedAtom) found for HandleTemporalPair = %s \n",
            //        htp.toString().c_str());
        }
        //} else {
        //    logger().debug("AtomSpace::getAtTimeLink: No TimeNode found for Temporal = %s (timeNodeName = %s)\n", t.toString().c_str(), t.getTimeNodeName().c_str());
    }
    return result;
}

const TruthValue& AtomSpace::getDefaultTV()
{
    return TruthValue::DEFAULT_TV();
}

Type AtomSpace::getTypeV(const tree<Vertex>& _target) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    return getType(boost::get<Handle>(*_target.begin()));
}

bool AtomSpace::isReal(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);
    return TLB::getAtom(h)->isReal();
}

Type AtomSpace::getType(Handle h) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
    return TLB::getAtom(h)->getType();
}

Type AtomSpace::getAtomType(const string& str) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return ClassServer::getType(const_cast<char*>(str.c_str()));
}

bool AtomSpace::inheritsType(Type t1, Type t2) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflus(stdout);

    // printf("AtomSpace::inheritsType(t1=%d,t2=%d)\n", t1, t2);
    bool result = ClassServer::isAssignableFrom(t2, t1);
    // printf("AtomSpace::inheritsType result = %d\n", result);
    return result;
}

bool AtomSpace::isNode(Type t) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    return inheritsType(t, NODE);
}

string AtomSpace::getName(Type t) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    if (ClassServer::getClassName()->find(t) == ClassServer::getClassName()->end()) {
        cassert(TRACE_INFO, false, "AtomSpace::getName(): Unknown atom type.");
    }
    return ClassServer::getTypeName(t);
}

#ifdef DEAD_CODE
bool AtomSpace::isNode(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);
    return inheritsType(getType(h), NODE);
}
#endif

bool AtomSpace::isVar(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);
    return inheritsType(getType(h), VARIABLE_NODE);
}

bool AtomSpace::isList(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);
    return inheritsType(getType(h), LIST_LINK);
}

bool AtomSpace::containsVar(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

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
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    Handle h = getHandle(t, str);
    return TLB::isValidHandle(h) ? h : addNode(t, str, TruthValue::NULL_TV());
}

Handle AtomSpace::createHandle(Type t, const HandleSeq& outgoing, bool managed)
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    Handle h = getHandle(t, outgoing);
    return TLB::isValidHandle(h) ? h : addLink(t, outgoing, TruthValue::NULL_TV());
}

bool AtomSpace::containsVersionedTV(Handle h, VersionHandle vh) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    bool result = isNullVersionHandle(vh);
    if (!result) {
        const TruthValue& tv = this->getTV(h);
        result = !tv.isNullTv() && tv.getType() == COMPOSITE_TRUTH_VALUE &&
                 !(((const CompositeTruthValue&) tv).getVersionedTV(vh).isNullTv());
    }
    return result;
}

Handle AtomSpace::addAtom(tree<Vertex>& a, tree<Vertex>::iterator it, const TruthValue& tvn)
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    cassert(TRACE_INFO, boost::get<Handle>(&(*it)) != NULL, "AtomSpace::addAtom(): Vertex should be of 'Handle' type.");

    HandleSeq handles;
    Handle head_type = boost::get<Handle>(*it);

    if (isReal(head_type)) {
        return addRealAtom(*(TLB::getAtom(head_type)), tvn);
    }

    for (tree<Vertex>::sibling_iterator i = a.begin(it); i != a.end(it); i++) {
        Handle *h_ptr = boost::get<Handle>(&*i);

        if (h_ptr && isReal(*h_ptr)) {
            handles.push_back(addRealAtom(*TLB::getAtom(*h_ptr), TruthValue::NULL_TV()));
        } else {
            handles.push_back(addAtom(a, i, TruthValue::TRIVIAL_TV()));
        }
    }

    return addLink((Type) ((long) TLB::getAtom(head_type)), handles, tvn);
}

Handle AtomSpace::addAtom(tree<Vertex>& a, const TruthValue& tvn)
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);
    return addAtom(a, a.begin(), tvn);
}

bool AtomSpace::removeAtom(Handle h, bool recursive)
{
    HandleEntry* extractedHandles = atomTable.extract(h, recursive);
    if (extractedHandles) {
        HandleEntry* currentEntry = extractedHandles;
        while (currentEntry) {
            Handle h = currentEntry->handle;

            // Also refund sti/lti to AtomSpace funds pool
            fundsSTI += getSTI(h);
            fundsLTI += getLTI(h);

            currentEntry = currentEntry->next;
        }
        atomTable.removeExtractedHandles(extractedHandles);
        return true;
    }
    return false;
}

bool TValued(Type t)
{
    return t != LIST_LINK;
}


const HandleSeq& AtomSpace::getOutgoing(Handle h) const
{
    static HandleSeq hs;
    Link *link = dynamic_cast<Link *>(TLB::getAtom(h));
    if (!link) return hs;
    return link->getOutgoingSet();
}

void AtomSpace::do_merge_tv(Handle h, const TruthValue& tvn)
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

Handle AtomSpace::addNode(Type t, const string& name, const TruthValue& tvn)
{
    Handle result = atomTable.getHandle(name.c_str(), t);
    if (TLB::isValidHandle(result))
    {
        // Just merges the TV
        if (!tvn.isNullTv()) do_merge_tv(result, tvn);
        return result;
    }

    // Remove default STI/LTI from AtomSpace Funds
    fundsSTI -= AttentionValue::DEFAULTATOMSTI;
    fundsLTI -= AttentionValue::DEFAULTATOMLTI;

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        result = backing_store->getHandle(t, name.c_str());
        if (TLB::isValidHandle(result))
        {
            if (!tvn.isNullTv()) do_merge_tv(result, tvn);
            return atomTable.add(TLB::getAtom(result));
        }
    }

    return atomTable.add(new Node(t, name, tvn));
}

Handle AtomSpace::addLink(Type t, const HandleSeq& outgoing,
                          const TruthValue& tvn)
{
    Handle result = atomTable.getHandle(t, outgoing);
    if (TLB::isValidHandle(result))
    {
        // Just merges the TV
        if (!tvn.isNullTv()) do_merge_tv(result, tvn);
        return result;
    }

    // Remove default STI/LTI from AtomSpace Funds
    fundsSTI -= AttentionValue::DEFAULTATOMSTI;
    fundsLTI -= AttentionValue::DEFAULTATOMLTI;

    // Maybe the backing store knows about this atom.
    if (backing_store)
    {
        result = backing_store->getHandle(t, outgoing);
        if (TLB::isValidHandle(result))
        {
            if (!tvn.isNullTv()) do_merge_tv(result, tvn);
            return atomTable.add(TLB::getAtom(result));
        }
    }

    return atomTable.add(new Link(t, outgoing, tvn));
}

Handle AtomSpace::addRealAtom(const Atom& atom, const TruthValue& tvn)
{
    //printf("AtomSpace::addRealAtom\n");
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

const string& AtomSpace::getName(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    Node * nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    if (nnn)
        return nnn->getName();
    else
        return emptyName;
}

Handle AtomSpace::getOutgoing(Handle h, int idx) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflus(stdout);
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

int AtomSpace::getArity(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflus(stdout);

    Atom * a = TLB::getAtom(h);
    Link * l = dynamic_cast<Link *>(a);
    if (NULL == l)
        return 0;
    return l->getArity();
}

void AtomSpace::setName(Handle h, const string& name)
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflus(stdout);

    Node *nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    cassert(TRACE_INFO, nnn != NULL, "AtomSpace::setName(): Handle h should be of 'Node' type.");
    nnn->setName(name);
}

HandleSeq AtomSpace::getIncoming(Handle h) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    HandleEntry* he = TLB::getAtom(h)->getIncomingSet();
    Handle *temp; int size;
    he->toHandleVector(temp, size);
    HandleSeq ret(size);
    for (int i = 0; i < size; i++)
        ret[i] = temp[i];
    return ret;
}

void AtomSpace::setTV(Handle h, const TruthValue& tv, VersionHandle vh)
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

const TruthValue& AtomSpace::getTV(Handle h, VersionHandle vh) const
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

void AtomSpace::setMean(Handle h, float mean) throw (InvalidParamException)
{
    TruthValue* newTv = getTV(h).clone();
    if (newTv->getType() == COMPOSITE_TRUTH_VALUE) {
        // Since CompositeTV has no setMean() method, we must handle it differently
        CompositeTruthValue* ctv = (CompositeTruthValue*) newTv;
        TruthValue* primaryTv = ctv->getVersionedTV(NULL_VERSION_HANDLE).clone();
        if (primaryTv->getType() == SIMPLE_TRUTH_VALUE) {
            ((SimpleTruthValue*)primaryTv)->setMean(mean);
        } else if (primaryTv->getType() == INDEFINITE_TRUTH_VALUE) {
            ((IndefiniteTruthValue*)primaryTv)->setMean(mean);
        } else {
            throw InvalidParamException(TRACE_INFO,
                                        "AtomSpace - Got a primaryTV with an invalid or unknown type");
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
                                        "AtomSpace - Got a TV with an invalid or unknown type");
        }
    }
    setTV(h, *newTv);
    delete newTv;
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
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    return TLB::getAtom(h)->getTruthValue().getCount();
}

int AtomSpace::Nodes(VersionHandle vh) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

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
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    atomTable.decayShortTermImportance();
}

long AtomSpace::getTotalSTI() const
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

long AtomSpace::getTotalLTI() const
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
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    /* This is too expensive and depending on an Agent that may be disabled.
     * Besides it does not have statistics by VersionHandles
     DynamicsStatisticsAgent *agent=DynamicsStatisticsAgent::getInstance();
     agent->reevaluateAllStatistics();
     return agent->getLinkCount();
     */
    // The following implementation is still expensive, but already deals with VersionHandles:
    HandleEntry* he = atomTable.getHandleSet(LINK, true, vh);
    int result = he->getSize();
    delete he;
    return result;
}

void AtomSpace::_getNextAtomPrepare()
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

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
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    _handle_entry = atomTable.getHandleSet(type, true);
}

Handle AtomSpace::_getNextAtom_type(Type type)
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflus(stdout);

    if (_handle_entry == NULL)
        return Handle::UNDEFINED;
    Handle h = _handle_entry->handle;
    _handle_entry = _handle_entry->next;
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
#if 0
    int j = 0;
    printf("%d nodes %d links to erase\n", Nodes(NULL_VERSION_HANDLE),
            Links(NULL_VERSION_HANDLE));
    printf("atoms in allAtoms: %d\n",allAtoms.size());
#endif 
    logger().enable();
    logger().setLevel(Logger::DEBUG);

    for (i = allAtoms.begin(); i != allAtoms.end(); i++) {
        result = removeAtom(*i,true);
#if 0
        if (result) {
            printf("%d: Atom %u removed, %d nodes %d links left to delete\n",
                j,*i,Nodes(NULL_VERSION_HANDLE), Links(NULL_VERSION_HANDLE));
            j++;
        }
#endif

    }

    allAtoms.clear();
    std::back_insert_iterator< std::vector<Handle> > output2(allAtoms);
    getHandleSet(output2, ATOM, true);
    assert(allAtoms.size() == 0);

}
