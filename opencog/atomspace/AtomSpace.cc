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
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

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
    backing_store = NULL;

    // XXX this is wrong, the space server should live in its own
    // directory, and not here. It needs to use the addAtom, removeAtom
    // signals to get its work done.
    spaceServer = new SpaceServer(*this);

    fundsSTI = config().get_int("STARTING_STI_FUNDS");
    fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;

    //connect signals
    addedAtomConnection = addAtomSignal().connect(boost::bind(&AtomSpace::atomAdded, this, _1));
    removedAtomConnection = removeAtomSignal().connect(boost::bind(&AtomSpace::atomRemoved, this, _1));

}

AtomSpace::~AtomSpace()
{
    //disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();


    // Check if has already been deleted. See in code where it can be delete.
    if (_handle_iterator) {
        delete _handle_iterator;
         _handle_iterator = NULL;
    }
    delete spaceServer;
    spaceServer = NULL;
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
            } else logger().warn("AtomSpace::addLink: Invalid atom type at the first element in an AtTimeLink's outgoing: %s\n", classserver().getTypeName(getType(timeNode)).c_str());
        } else logger().warn("AtomSpace::addLink: Invalid arity for an AtTimeLink: %d (expected: 2)\n", getArity(h));
    }
}

void AtomSpace::atomRemoved(Handle h)
{
    Type type = getType(h);
    if (type == AT_TIME_LINK)
    {
        OC_ASSERT(getArity(h) == 2, "AtomSpace::atomRemoved: Got invalid arity for removed AtTimeLink = %d\n", getArity(h));
        Handle timeNode = getOutgoing(h, 0);
        OC_ASSERT(getType(timeNode) == TIME_NODE, "AtomSpace::atomRemoved: Got no TimeNode node at the first position of the AtTimeLink\n");
        Handle timedAtom = getOutgoing(h, 1);
        timeServer.remove(timedAtom, Temporal::getFromTimeNodeName(((Node*) TLB::getAtom(timeNode))->getName().c_str()));

        // XXX THIS IS WRONG --- instead, the space server
        // should listen for the removeAtomSignal signal, and go with that.
        // if outgoingSet[1] is a SpaceMap concept node, remove related map from SpaceServer
        if( getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME) == timedAtom ){
           spaceServer->removeMap(h);
        } // if
    } else if ( inheritsType(type, OBJECT_NODE) ) {
        spaceServer->removeObject(getName(h));
    } // else if
}

// ====================================================================

const AtomTable& AtomSpace::getAtomTable() const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);
    return atomTable;
}

const TimeServer& AtomSpace::getTimeServer() const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);

    return timeServer;
}

SpaceServer& AtomSpace::getSpaceServer() const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);

    return *spaceServer;
}


void AtomSpace::print(std::ostream& output, Type type, bool subclass) const
{
    atomTable.print(output, type, subclass);
}

Handle AtomSpace::addTimeInfo(Handle h, unsigned long timestamp, const TruthValue& tv)
{
    OC_ASSERT(TLB::isValidHandle(h), "AtomSpace::addTimeInfo: Got an invalid handle as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const Temporal& t, const TruthValue& tv)
{
    OC_ASSERT(TLB::isValidHandle(h), "AtomSpace::addTimeInfo: Got an invalid handle as argument\n");
    OC_ASSERT(t != UNDEFINED_TEMPORAL, "AtomSpace::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    return addTimeInfo(h, t.getTimeNodeName(), tv);
}

Handle AtomSpace::addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv)
{
    //logger().debug("AtomSpace::addTimeInfo - temp init");
    Handle timeNode = addNode(TIME_NODE, timeNodeName.c_str());
    //logger().debug("AtomSpace::addTimeInfo - temp 1");
    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    atTimeLinkOutgoing.push_back(h);
    Handle atTimeLink = addLink(AT_TIME_LINK, atTimeLinkOutgoing, tv);
    //logger().debug("AtomSpace::addTimeInfo - temp end");
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
            OC_ASSERT(getArity(atTimeLink) == 2, "AtomSpace::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n", getArity(atTimeLink));
            OC_ASSERT(TLB::isValidHandle(timeNode) && getType(timeNode) == TIME_NODE, "AtomSpace::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
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

bool AtomSpace::addSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                              double objX, double objY, double objZ,
                              double objLength, double objWidth, double objHeight,
                              double objYaw, bool isObstacle) {

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = addTimeInfo(spaceMapNode, timestamp);
    bool result =  spaceServer->add( keepPreviousMap, spaceMapAtTimeLink, getName(objectNode),
                        objX, objY, objZ, objLength, objWidth, objHeight, objYaw, isObstacle);

    return result;
}

Handle AtomSpace::addSpaceMap(unsigned long timestamp, SpaceServer::SpaceMap * spaceMap){

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = addTimeInfo(spaceMapNode, timestamp);
    spaceServer->add(spaceMapAtTimeLink, spaceMap);

    return spaceMapAtTimeLink;
}

Handle AtomSpace::removeSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp) {

    logger().debug("%s(%s)\n", __FUNCTION__, getName(objectNode).c_str());

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = addTimeInfo(spaceMapNode, timestamp);
    spaceServer->remove(keepPreviousMap, spaceMapAtTimeLink, getName(objectNode));

    return spaceMapAtTimeLink;
}

void AtomSpace::cleanupSpaceServer(){

    // sanity checks
    if (spaceServer->getSpaceMapsSize() < 1) {
        logger().debug(
                       "AtomSpace - No need to clean SpaceServer. It has no space map yet.");
        return;
    }

    // sanity tests passed, cleaning SpaceServer
    Handle spaceMapNode = getSpaceMapNode();

    // get all HandleTemporalPairs associated with the SpaceMap concept node.
    std::vector<HandleTemporalPair> pairs;
    getTimeInfo(back_inserter(pairs), spaceMapNode);

    int j = 0;
    // remember to leave at least one map in SpaceServer, the newer one.
    for(unsigned int i = 0; i < pairs.size() - 1; i++){

        // get SpaceMap handles
        Handle mapHandle = getAtTimeLink(pairs[i]);

        // mapHandle not among the ones that should be preserved
        if (!spaceServer->containsMap(mapHandle) || !spaceServer->isMapPersistent(mapHandle)){
            j++;
            logger().debug("AtomSpace - Removing map (%s)", TLB::getAtom(mapHandle)->toString().c_str());
            // remove map from SpaceServer, and timeInfo from TimeServer and AtomSpace
            removeAtom(mapHandle, true);
        }
    }
    logger().debug("AtomSpace - Number of deleted maps: %d.", j);
}

void AtomSpace::mapRemoved(Handle mapId)
{
    // Remove this atom from AtomSpace since its map does not exist anymore 
    removeAtom(mapId);
}

void AtomSpace::mapPersisted(Handle mapId)
{
    // set LTI to a value that prevents the corresponding atom to be removed
    // from AtomSpace
    setLTI(mapId, 1);
}

std::string AtomSpace::getMapIdString(Handle mapHandle)
{
    // Currently the mapHandle is of AtTimeLink(TimeNode:"<timestamp>" , ConceptNode:"SpaceMap")
    // So, just get the name of the TimeNode as its string representation
    return getName(getOutgoing(mapHandle, 0));
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

Type AtomSpace::getType(Handle h) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);
    return TLB::getAtom(h)->getType();
}

Type AtomSpace::getAtomType(const string& str) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);

    return classserver().getType(const_cast<char*>(str.c_str()));
}

bool AtomSpace::inheritsType(Type t1, Type t2) const
{
    // fprintf(stdout,"Atom space address: %p\n", this);
    // fflush(stdout);

    // printf("AtomSpace::inheritsType(t1=%d,t2=%d)\n", t1, t2);
    bool result = classserver().isA(t1, t2);
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
    //fflush(stdout);

    /*
    if (ClassServer::getClassName()->find(t) == ClassServer::getClassName()->end()) {
        OC_ASSERT(false, "AtomSpace::getName(): Unknown atom type.");
    }
    */
    return classserver().getTypeName(t);
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
    Node * nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    if (nnn)
        return nnn->getName();
    else
        return emptyName;
}

Handle AtomSpace::getOutgoing(Handle h, int idx) const
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

int AtomSpace::getArity(Handle h) const
{
    Atom * a = TLB::getAtom(h);
    Link * l = dynamic_cast<Link *>(a);
    if (NULL == l)
        return 0;
    return l->getArity();
}

void AtomSpace::setName(Handle h, const string& name)
{
    Node *nnn = dynamic_cast<Node*>(TLB::getAtom(h));
    OC_ASSERT(nnn != NULL, "AtomSpace::setName(): Handle h should be of 'Node' type.");
    nnn->setName(name);
}

HandleSeq AtomSpace::getIncoming(Handle h)
{
    // Ugh. It is possible that the incoming set that we currently 
    // hold is much smaller than what is in storage. In this case,
    // we would like to automatically pull all of those other atoms
    // into here (using fetchIncomingSet(h,true) to do so). However,
    // maybe the incoming set is up-to-date, in which case polling 
    // storage over and over is a huge waste of time.  What to do? 
    //
    // h = fetchIncomingSet(h, true);

    HandleEntry* he = TLB::getAtom(h)->getIncomingSet();
    return he->toHandleVector();
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
    //fflush(stdout);

    return TLB::getAtom(h)->getTruthValue().getCount();
}

int AtomSpace::Nodes(VersionHandle vh) const
{
    //fprintf(stdout,"Atom space address: %p\n", this);
    //fflush(stdout);

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
    //fflush(stdout);

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
    //fflush(stdout);

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
