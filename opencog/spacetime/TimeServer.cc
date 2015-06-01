/*
 * opencog/spacetime/TimeServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/spacetime/atom_types.h>
#include "TimeServer.h"

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

// USED TO SEEK MEMORY LEAK
//int TimeServer::timeServerEntries = 0;
//std::set<Temporal> TimeServer::temporalSet;

void TimeServer::init()
{
    table = new TemporalTable();
    latestTimestamp = 0;
}

TimeServer::TimeServer(AtomSpace& a, SpaceServer *_ss)
   : atomspace(&a), spaceServer(_ss)
{
    init();
    spaceServer->setTimeServer(this);

    // Connect signals
    addedAtomConnection = a.addAtomSignal(boost::bind(&TimeServer::atomAdded, this, _1));
    removedAtomConnection = a.removeAtomSignal(boost::bind(&TimeServer::atomRemoved, this, _1));
}

TimeServer::~TimeServer()
{
    // disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();
    delete table;
}

void TimeServer::add(Handle h, const Temporal& t)
{
    // USED TO SEEK MEMORY LEAK
    //++timeServerEntries;
    //cout << "Total timeServerEntries: " << timeServerEntries << endl;

    //if(temporalSet.find(t) == temporalSet.end()){
    //   temporalSet.insert(t);
    //   cout << "Total unique entrys: " << temporalSet.size() << endl;
    //}
    std::unique_lock<std::mutex> lock(ts_mutex);
    table->add(h, t);
    if (t.getUpperBound() > latestTimestamp) {
        latestTimestamp = t.getUpperBound();
    }
}

bool TimeServer::remove(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion)
{
    std::unique_lock<std::mutex>  lock(ts_mutex);
    return table->remove(h, t, criterion);
}

octime_t TimeServer::getLatestTimestamp() const
{
    std::unique_lock<std::mutex> lock(ts_mutex);
    return latestTimestamp;
}

TimeServer& TimeServer::operator=(const TimeServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "TimeServer - Cannot copy an object of this class");
}

TimeServer::TimeServer(const TimeServer& other) 
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "TimeServer - Cannot copy an object of this class");
}

void TimeServer::clear()
{
    std::unique_lock<std::mutex> lock(ts_mutex);
    delete table;
    init();
}

Handle TimeServer::addTimeInfo(Handle h, octime_t timestamp, TruthValuePtr tv)
{
    OC_ASSERT(atomspace->isValidHandle(h),
            "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, tv);
}

Handle TimeServer::addTimeInfo(Handle h, const Temporal& t, TruthValuePtr tv)
{
    OC_ASSERT(atomspace->isValidHandle(h),
            "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    OC_ASSERT(t != UNDEFINED_TEMPORAL, "TimeServer::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    return addTimeInfo(h, t.getTimeNodeName(), tv);
}

Handle TimeServer::addTimeInfo(Handle h, const std::string& timeNodeName, TruthValuePtr tv)
{
    DPRINTF("TimeServer::addTimeInfo - start\n");
    Handle timeNode = atomspace->addNode(TIME_NODE, timeNodeName);
    DPRINTF("TimeServer::addTimeInfo - timeNode was %lu\n", timeNode.value());
    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    atTimeLinkOutgoing.push_back(h);
    Handle atTimeLink = atomspace->addLink(AT_TIME_LINK, atTimeLinkOutgoing);
    atTimeLink->setTruthValue(tv);
    DPRINTF("TimeServer::addTimeInfo - atTimeLink was %lu\n", atTimeLink.value());
    DPRINTF("TimeServer::addTimeInfo - temp end\n");
    return atTimeLink;
}

bool TimeServer::removeTimeInfo(Handle h, octime_t timestamp, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive)
{
    Temporal t(timestamp);
    return removeTimeInfo(h, t, criterion, removeDisconnectedTimeNodes, recursive);
}

bool TimeServer::removeTimeInfo(Handle h, const Temporal& t,
        TemporalTable::TemporalRelationship criterion,
        bool removeDisconnectedTimeNodes, bool recursive)
{
    DPRINTF("TimeServer::removeTimeInfo(%s, %s, %s, %d, %d)\n", atomspace->atomAsString(h).c_str(), t.toString().c_str(), TemporalTable::getTemporalRelationshipStr(criterion), removeDisconnectedTimeNodes, recursive);

    std::list<HandleTemporalPair> existingEntries;
    get(back_inserter(existingEntries), h, t, criterion);
    bool result = !existingEntries.empty();
    for (std::list<HandleTemporalPair>::const_iterator itr = existingEntries.begin();
            itr != existingEntries.end(); ++itr) {
        Handle atTimeLink = getAtTimeLink(*itr);
        DPRINTF("Got atTimeLink = %lu\n", atTimeLink.value());
        if (atomspace->isValidHandle(atTimeLink)) {
            Handle timeNode = atomspace->getOutgoing(atTimeLink, 0);
            DPRINTF("Got timeNode = %lu\n", timeNode.value());
            int arityOfTimeLink = atomspace->getArity(atTimeLink);
            OC_ASSERT(arityOfTimeLink == 2,
                    "TimeServer::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n",
                    arityOfTimeLink);
            OC_ASSERT(atomspace->isValidHandle(timeNode)
                    && atomspace->getType(timeNode) == TIME_NODE,
                    "TimeServer::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
            if (atomspace->removeAtom(atTimeLink, recursive)) {
                DPRINTF("atTimeLink removed from AT successfully\n");
                if (removeDisconnectedTimeNodes &&
                        atomspace->getIncoming(timeNode).empty()) {
                    DPRINTF("Trying to remove timeNode as well\n");
                    atomspace->removeAtom(timeNode);
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

Handle TimeServer::getAtTimeLink(const HandleTemporalPair& htp) const
{
    const Temporal& t = *(htp.getTemporal());
    Handle h = htp.getHandle();
    DPRINTF("TimeServer::getAtTimeLink: t = %s, h = %s\n", t.toString().c_str(), atomspace->atomAsString(h).c_str());

    Handle timeNode = atomspace->getHandle(TIME_NODE, t.getTimeNodeName());
    DPRINTF("timeNode = %lu\n", timeNode.value());
    if (atomspace->isValidHandle(timeNode)) {
        return atomspace->getHandle(AT_TIME_LINK, timeNode, h);
    }
    return Handle::UNDEFINED;
}

void TimeServer::atomAdded(Handle h)
{
    Type type = h->getType();
    if (type == AT_TIME_LINK) {
        // Add corresponding TimeServer entry
        LinkPtr lll = LinkCast(h);
        if (lll->getArity() == 2) {
            Handle timeNode = lll->getOutgoingAtom(0);
            if (timeNode->getType() == TIME_NODE) {
                NodePtr nnn = NodeCast(timeNode);
                const string& timeNodeName = nnn->getName();
                Temporal t = Temporal::getFromTimeNodeName(timeNodeName.c_str());
                Handle timed_h = lll->getOutgoingAtom(1);
                add(timed_h, t);
            } else logger().warn("TimeServer::atomAdded: Invalid atom type "
                    "at the first element in an AtTimeLink's outgoing: "
                    "%s\n", classserver().getTypeName(timeNode->getType()).c_str());
        } else logger().warn("TimeServer::atomAdded: Invalid arity for an "
                "AtTimeLink: %d (expected: 2)\n", lll->getArity());
    }
}

void TimeServer::atomRemoved(AtomPtr atom)
{
    Type type = atom->getType();
    if (type != AT_TIME_LINK) return;

    LinkPtr lll(LinkCast(atom));
    OC_ASSERT(lll->getArity() == 2,
        "AtomSpace::atomRemoved: Got invalid arity for removed AtTimeLink = %d\n",
        lll->getArity());

    AtomPtr timeNode = lll->getOutgoingAtom(0);
 
    // If it's not a TimeNode, then it's a VariableNode which can stand
    // in for a TimeNode. So we can ignore it here.
    if (timeNode->getType() != TIME_NODE) return;

    AtomPtr timedAtom = lll->getOutgoingAtom(1);
    
#if DOES_NOT_COMPILE_RIGHT_NOW
    // We have to do the check here instead of in the spaceServer
    // if outgoingSet[1] is a SpaceMap concept node, remove related map from SpaceServer
    if (a->getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME) == timedAtom->getHandle())
       spaceServer->removeMap(atom->getHandle());
#endif
    NodePtr nnn(NodeCast(timeNode));
    remove(timedAtom->getHandle(), Temporal::getFromTimeNodeName(nnn->getName().c_str()));
}
