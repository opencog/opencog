/*
 * opencog/atomspace/TimeServer.cc
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

#include "TimeServer.h"

#include <opencog/util/Logger.h>

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

TimeServer::TimeServer(AtomSpaceAsync& a, SpaceServer *_ss) : atomspace(&a), spaceServer(_ss)
{
    init();
    // Connect signals
    a.addAtomSignal(boost::bind(&TimeServer::atomAdded, this, _1, _2));
    a.removeAtomSignal(boost::bind(&TimeServer::atomRemoved, this, _1, _2));
}

TimeServer::~TimeServer()
{
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
    boost::mutex::scoped_lock lock(ts_mutex);
    table->add(h, t);
    if (t.getUpperBound() > latestTimestamp) {
        latestTimestamp = t.getUpperBound();
    }
}

bool TimeServer::remove(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion)
{
    boost::mutex::scoped_lock lock(ts_mutex);
    return table->remove(h, t, criterion);
}

unsigned long TimeServer::getLatestTimestamp() const
{
    boost::mutex::scoped_lock lock(ts_mutex);
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
    boost::mutex::scoped_lock lock(ts_mutex);
    delete table;
    init();
}

Handle TimeServer::addTimeInfo(Handle h, unsigned long timestamp, const TruthValue& tv)
{
    OC_ASSERT(atomspace->isValidHandle(h)->get_result(),
            "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, tv);
}

Handle TimeServer::addTimeInfo(Handle h, const Temporal& t, const TruthValue& tv)
{
    OC_ASSERT(atomspace->isValidHandle(h)->get_result(),
            "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    OC_ASSERT(t != UNDEFINED_TEMPORAL, "TimeServer::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    return addTimeInfo(h, t.getTimeNodeName(), tv);
}

Handle TimeServer::addTimeInfo(Handle h, const std::string& timeNodeName, const TruthValue& tv)
{
    DPRINTF("TimeServer::addTimeInfo - start\n");
    Handle timeNode = atomspace->addNode(TIME_NODE, timeNodeName)->get_result();
    DPRINTF("TimeServer::addTimeInfo - timeNode was %lu\n", timeNode.value());
    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    atTimeLinkOutgoing.push_back(h);
    Handle atTimeLink = atomspace->addLink(AT_TIME_LINK, atTimeLinkOutgoing, tv)->get_result();
    DPRINTF("TimeServer::addTimeInfo - atTimeLink was %lu\n", atTimeLink.value());
    DPRINTF("TimeServer::addTimeInfo - temp end\n");
    return atTimeLink;
}

bool TimeServer::removeTimeInfo(Handle h, unsigned long timestamp, TemporalTable::TemporalRelationship criterion, bool removeDisconnectedTimeNodes, bool recursive)
{
    Temporal t(timestamp);
    return removeTimeInfo(h, t, criterion, removeDisconnectedTimeNodes, recursive);
}

bool TimeServer::removeTimeInfo(Handle h, const Temporal& t,
        TemporalTable::TemporalRelationship criterion,
        bool removeDisconnectedTimeNodes, bool recursive)
{
    DPRINTF("TimeServer::removeTimeInfo(%s, %s, %s, %d, %d)\n", atomspace->atomAsString(h)->get_result().c_str(), t.toString().c_str(), TemporalTable::getTemporalRelationshipStr(criterion), removeDisconnectedTimeNodes, recursive);

    std::list<HandleTemporalPair> existingEntries;
    get(back_inserter(existingEntries), h, t, criterion);
    bool result = !existingEntries.empty();
    for (std::list<HandleTemporalPair>::const_iterator itr = existingEntries.begin();
            itr != existingEntries.end(); ++itr) {
        Handle atTimeLink = getAtTimeLink(*itr);
        DPRINTF("Got atTimeLink = %lu\n", atTimeLink.value());
        if (atomspace->isValidHandle(atTimeLink)->get_result()) {
            Handle timeNode = atomspace->getOutgoing(atTimeLink, 0)->get_result();
            DPRINTF("Got timeNode = %lu\n", timeNode.value());
            int arityOfTimeLink = atomspace->getArity(atTimeLink)->get_result();
            OC_ASSERT(arityOfTimeLink == 2,
                    "TimeServer::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n",
                    arityOfTimeLink);
            OC_ASSERT(atomspace->isValidHandle(timeNode)->get_result()
                    && atomspace->getType(timeNode)->get_result() == TIME_NODE,
                    "TimeServer::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
            if (atomspace->removeAtom(atTimeLink, recursive)->get_result()) {
                DPRINTF("atTimeLink removed from AT successfully\n");
                if (removeDisconnectedTimeNodes &&
                        atomspace->getIncoming(timeNode)->get_result().empty()) {
                    DPRINTF("Trying to remove timeNode as well\n");
                    atomspace->removeAtom(timeNode)->get_result();
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
    Handle result = Handle::UNDEFINED;

    const Temporal& t = *(htp.getTemporal());
    Handle h = htp.getHandle();
    DPRINTF("TimeServer::getAtTimeLink: t = %s, h = %s\n", t.toString().c_str(), atomspace->atomAsString(h)->get_result().c_str());

    Handle timeNode = atomspace->getHandle(TIME_NODE, t.getTimeNodeName())->get_result();
    DPRINTF("timeNode = %lu\n", timeNode.value());
    if (atomspace->isValidHandle(timeNode)->get_result()) {
        HandleSeq atTimeLinkOutgoing(2);
        atTimeLinkOutgoing[0] = timeNode;
        atTimeLinkOutgoing[1] = h;
        HandleSeq atTimeLinks = atomspace->getHandlesByOutgoingSet(atTimeLinkOutgoing,
                NULL, NULL, 2, AT_TIME_LINK, false)->get_result();
        if (!atTimeLinks.empty()) {
            result = atTimeLinks[0];
            if (atTimeLinks.size() > 1) {
                logger().warn(
                    "TimeServer::getAtTimeLink: More than 1 AtTimeLink(TimeNode, TimedAtom) found for HandleTemporalPair = %s \n",
                    htp.toString().c_str());
            }
        }
    }
    return result;
}

void TimeServer::atomAdded(AtomSpaceImpl* a, Handle h)
{
    Type type = a->getType(h);
    if (type == AT_TIME_LINK) {
        // Add corresponding TimeServer entry
        if (a->getArity(h) == 2) {
            Handle timeNode = a->getOutgoing(h, 0);
            if (a->getType(timeNode) == TIME_NODE) {
                const string& timeNodeName = a->getName(timeNode);
                Temporal t = Temporal::getFromTimeNodeName(timeNodeName.c_str());
                Handle timed_h = a->getOutgoing(h, 1);
                add(timed_h, t);
            } else logger().warn("TimeServer::atomAdded: Invalid atom type "
                    "at the first element in an AtTimeLink's outgoing: "
                    "%s\n", classserver().getTypeName(a->getType(timeNode)).c_str());
        } else logger().warn("TimeServer::atomAdded: Invalid arity for an "
                "AtTimeLink: %d (expected: 2)\n", a->getArity(h));
    }
}

void TimeServer::atomRemoved(AtomSpaceImpl* a, Handle h)
{
    Type type = a->getType(h);
    if (type == AT_TIME_LINK)
    {
        OC_ASSERT(a->getArity(h) == 2, "AtomSpace::atomRemoved: Got invalid arity for removed AtTimeLink = %d\n", a->getArity(h));
        Handle timeNode = a->getOutgoing(h, 0);
        OC_ASSERT(a->getType(timeNode) == TIME_NODE, "AtomSpace::atomRemoved: Got no TimeNode node at the first position of the AtTimeLink\n");
        Handle timedAtom = a->getOutgoing(h, 1);
        remove(timedAtom, Temporal::getFromTimeNodeName(a->getName(timeNode).c_str()));

        // We have to do the check here instead of in the spaceServer
        // if outgoingSet[1] is a SpaceMap concept node, remove related map from SpaceServer
        if( a->getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME) == timedAtom ){
           spaceServer->removeMap(h);
        }
    }
}

