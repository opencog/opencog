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
TimeDomain opencog::DEFAULT_TIMEDOMAIN ="Default TimeDomain";

void TimeServer::init()
{
    latestTimestamp = 0;
    //TODO: Add a UTC time entry. It might be useful for syncronizing
    // the times in multiple atomspaces.
    // Eg. use case: suppose multiple robots/virtual-agents/IOT collect time
    // series data merging of these data, for analysis/mining.
}

TimeServer::TimeServer(AtomSpace& a): atomspace(&a), spaceServer(NULL)
{
    init();
    // Connect signals
    addedAtomConnection = a.addAtomSignal(boost::bind(&TimeServer::atomAdded, this, _1));
    removedAtomConnection = a.removeAtomSignal(boost::bind(&TimeServer::atomRemoved, this, _1));
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
}

void TimeServer::add(Handle h, const Temporal& t, const TimeDomain& timeDomain)
{
    // USED TO SEEK MEMORY LEAK
    //++timeServerEntries;
    //cout << "Total timeServerEntries: " << timeServerEntries << endl;

    //if(temporalSet.find(t) == temporalSet.end()){
    //   temporalSet.insert(t);
    //   cout << "Total unique entrys: " << temporalSet.size() << endl;
    //
    std::unique_lock<std::mutex> lock(ts_mutex);

    temporalTableMap[timeDomain].add(h,t);

    if (t.getUpperBound() > latestTimestamp) {
        latestTimestamp = t.getUpperBound();
    }

}

bool TimeServer::remove(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion, const TimeDomain& timeDomain)
{
    auto temporalTableIter = temporalTableMap.find(timeDomain);
    if (temporalTableIter == temporalTableMap.end()) {
        logger().error("TimeServer::remove: timedomain %s not found\n", timeDomain.c_str());
        return false;
    }

    std::unique_lock<std::mutex> lock(ts_mutex);
    return (temporalTableIter->second).remove(h, t, criterion);
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
    temporalTableMap.clear();
    init();
}

Handle TimeServer::addTimeInfo(Handle h, const octime_t& timestamp, const TimeDomain& timeDomain, TruthValuePtr tv )
{
    OC_ASSERT(atomspace->is_valid_handle(h),
              "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    std::string nodeName = Temporal::getTimeNodeName(timestamp);
    return addTimeInfo(h, nodeName, timeDomain, tv);
}

Handle TimeServer::addTimeInfo(Handle h, const Temporal& t, const TimeDomain& timeDomain, TruthValuePtr tv)
{
    OC_ASSERT(atomspace->is_valid_handle(h),
	      "TimeServer::addTimeInfo: Got an invalid handle as argument\n");
    OC_ASSERT(t != UNDEFINED_TEMPORAL, "TimeServer::addTimeInfo: Got an UNDEFINED_TEMPORAL as argument\n");
    this->add(h, t, timeDomain);
    return addTimeInfo(h, t.getTimeNodeName(), timeDomain, tv);
}

Handle TimeServer::addTimeInfo(Handle h, const std::string& timeNodeName, const TimeDomain& timeDomain, TruthValuePtr tv)
{
    DPRINTF("TimeServer::addTimeInfo - start\n");
    Handle timeNode = atomspace->add_node(TIME_NODE, timeNodeName);

    HandleSeq atTimeLinkOutgoing;
    atTimeLinkOutgoing.push_back(timeNode);
    DPRINTF("TimeServer::addTimeInfo - timeNode was %lu\n", timeNode.value());
    atTimeLinkOutgoing.push_back(h);

    // If it's default time domain, it means that we only have a single time domain.
    // So we'll not have TimeDomainNode;
    // Otherwise we will add TimeDomainNode.
    if (timeDomain != DEFAULT_TIMEDOMAIN) {
        Handle timeDomainNode = atomspace->add_node(TIME_DOMAIN_NODE, timeDomain);
        DPRINTF("TimeServer::addTimeInfo - timeDomainNode was %lu\n", timeDomainNode.value());
        atTimeLinkOutgoing.push_back(timeDomainNode);
    }

    Handle atTimeLink = atomspace->add_link(AT_TIME_LINK, atTimeLinkOutgoing);
    atTimeLink->setTruthValue(tv);
    DPRINTF("TimeServer::addTimeInfo - atTimeLink was %lu\n", atTimeLink.value());
    return atTimeLink;
}

bool TimeServer::removeTimeInfo(Handle h,
                                const octime_t& timestamp,
                                TemporalTable::TemporalRelationship criterion,
                                const TimeDomain& timeDomain,
                                bool removeDisconnectedTimeNodes,
                                bool recursive)
{
    Temporal t(timestamp);
    return removeTimeInfo(h, t, criterion, timeDomain, removeDisconnectedTimeNodes, recursive);
}

bool TimeServer::removeTimeInfo(Handle h,
                                const Temporal& t,
                                TemporalTable::TemporalRelationship criterion,
                                const TimeDomain& timeDomain,
                                bool removeDisconnectedTimeNodes,
                                bool recursive)
{
    DPRINTF("TimeServer::removeTimeInfo(%s, %s, %s, %d, %d)\n", atomspace->atom_as_string(h).c_str(), t.toString().c_str(), TemporalTable::getTemporalRelationshipStr(criterion), removeDisconnectedTimeNodes, recursive);

    std::list<HandleTemporalPair> existingEntries;
    get(back_inserter(existingEntries), h, t, criterion, timeDomain);
    bool result = !existingEntries.empty();
    for (std::list<HandleTemporalPair>::const_iterator itr = existingEntries.begin();
		 itr != existingEntries.end(); ++itr)
    {
        Handle atTimeLink = getAtTimeLink(*itr, timeDomain);
        DPRINTF("Got atTimeLink = %lu\n", atTimeLink.value());
        if (atomspace->is_valid_handle(atTimeLink)) {
	    Handle timeNode = LinkCast(atTimeLink)->getOutgoingAtom(0);
            DPRINTF("Got timeNode = %lu\n", timeNode.value());
            OC_ASSERT(atomspace->is_valid_handle(timeNode)
                      and timeNode->getType() == TIME_NODE,
                      "TimeServer::removeTimeInfo: Got no TimeNode node at the first position of the AtTimeLink\n");
            int arityOfTimeLink = LinkCast(atTimeLink)->getArity();

            if (timeDomain == DEFAULT_TIMEDOMAIN) {
                //single time domain; should have 2 arities
                OC_ASSERT(arityOfTimeLink == 2,
                          "TimeServer::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n",
                          arityOfTimeLink);
            }
            else {
                //Multiple time domains; should have 3 arities
                OC_ASSERT(arityOfTimeLink == 3,
                          "TimeServer::removeTimeInfo: Got invalid arity for AtTimeLink = %d\n",
                          arityOfTimeLink);
            }

            if (atomspace->remove_atom(atTimeLink, recursive)) {
                DPRINTF("atTimeLink removed from AT successfully\n");
                if (removeDisconnectedTimeNodes and
                    timeNode->getIncomingSetSize() == 0)
                {
                    DPRINTF("Trying to remove timeNode as well\n");
                    atomspace->remove_atom(timeNode);
                }
            }
	    else {
                result = false;
            }
        }
	else{
            result = false;
        }
    }
    return result;
}

Handle TimeServer::getAtTimeLink(const HandleTemporalPair& htp, const TimeDomain& timeDomain) const
{
    const Temporal& t = *(htp.getTemporal());
    Handle h = htp.getHandle();
    DPRINTF("TimeServer::getAtTimeLink: t = %s, h = %s\n", t.toString().c_str(), atomspace->atom_as_string(h).c_str());

    Handle timeNode = atomspace->get_handle(TIME_NODE, t.getTimeNodeName());
    HandleSeq outgoing;
    outgoing.push_back(timeNode);
    outgoing.push_back(h);
    if (timeDomain != DEFAULT_TIMEDOMAIN) {
        //multiple time domain; should add TimeDomainNode in the last arity
        Handle timeDomainNode = atomspace->get_handle(TIME_DOMAIN_NODE, timeDomain);
        outgoing.push_back(timeDomainNode);
    }
    DPRINTF("timeNode = %lu\n", timeNode.value());
    if (atomspace->is_valid_handle(timeNode)) {
        return atomspace->get_handle(AT_TIME_LINK, outgoing);
    }
    return Handle::UNDEFINED;
}
/*
TimeDomain TimeServer::getTimeDomain() const
{
    vector<string> timeDomains = getTimeDomains();
    string timeDomain;
    if (timeDomains.empty()) {
        timeDomain = "DefaultTimeDomain";
    }
    else if (timeDomains.size() >= 1) {
	if (timeDomains.size() > 1) {
	    logger().error("TimeServer::getTimeDomain: There's more than one time domain in TimeServer. we will use the first searched timeDomain name %s. If you don't want this behavior,just change the code.", timeDomains[0].c_str());
	}
	timeDomain = timeDomains[0];
    }
    return timeDomain;
}
*/
vector<TimeDomain> TimeServer::getTimeDomains() const
{
    vector<TimeDomain> result;
    for (auto timeDomainTablePair : temporalTableMap) {
        result.push_back(timeDomainTablePair.first);
    }
    return result;
}

void TimeServer::atomAdded(Handle h)
{
    Type type = h->getType();
    if (type != AT_TIME_LINK) return;

    int arityOfTimeLink = h->getArity();
    if (arityOfTimeLink != 2 and arityOfTimeLink != 3)
    {
        logger().warn("TimeServer::atomAdded: Invalid arity for an "
            "AtTimeLink: %d (expected: 2 (for default time domain) "
            "or 3 (for multiple time domains))\n", arityOfTimeLink);
        return;
    }

    // Add corresponding TimeServer entry
    const Handle& timeNode = h->getOutgoingAtom(0);
    if (timeNode->getType() == TIME_NODE) {
        const string& timeNodeName = timeNode->getName();
        const Handle& timed_h = h->getOutgoingAtom(1);
        Temporal t = Temporal::getFromTimeNodeName(timeNodeName.c_str());
        TimeDomain timeDomain = DEFAULT_TIMEDOMAIN;
        if (arityOfTimeLink == 3) {
            const Handle& timeDomainNode = h->getOutgoingAtom(2);
            if (timeDomainNode->getType() == TIME_DOMAIN_NODE) {
                timeDomain = timeDomainNode->getName();
            }
            else {
                logger().warn("TimeServer::atomAdded: Invalid atom type "
                      "at the third element in an AtTimeLink's outgoing: "
                      "%s\n",
                      classserver().getTypeName(timeDomainNode->getType()).c_str());
                return;
            }
        }
        add(timed_h, t, timeDomain);
    } else {
        logger().warn("TimeServer::atomAdded: Invalid atom type "
             "at the first element in an AtTimeLink's outgoing: "
             "%s\n",
             classserver().getTypeName(timeNode->getType()).c_str());
    }
}

void TimeServer::atomRemoved(AtomPtr atom)
{
    Type type = atom->getType();
    if (type != AT_TIME_LINK) {
        return;
    }

    LinkPtr lll(LinkCast(atom));
    int arityOfTimeLink = lll->getArity();
    OC_ASSERT(arityOfTimeLink != 3 || arityOfTimeLink != 2,
	      "AtomSpace::atomRemoved: Got invalid arity for removed AtTimeLink = %d\n",
	      arityOfTimeLink);

    AtomPtr timeNode = lll->getOutgoingAtom(0);
    // If it's not a TimeNode, then it's a VariableNode which can stand
    // in for a TimeNode. So we can ignore it here.
    if (timeNode->getType() != TIME_NODE) {
        return;
    }
    AtomPtr timedAtom = lll->getOutgoingAtom(1);

    TimeDomain timeDomain = DEFAULT_TIMEDOMAIN;
    if (arityOfTimeLink == 3) {
        AtomPtr timeDomainNode = lll->getOutgoingAtom(2);
        if (timeNode->getType() != TIME_DOMAIN_NODE) {
            return;
        }
        timeDomain = NodeCast(timeDomainNode)->getName();
    }

#if DOES_NOT_COMPILE_RIGHT_NOW
    // We have to do the check here instead of in the spaceServer
    // if outgoingSet[1] is a SpaceMap concept node, remove related map from SpaceServer
    if (a->getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME) == timedAtom->getHandle())
        spaceServer->removeMap(atom->getHandle());
#endif
    NodePtr nnn(NodeCast(timeNode));
    remove(timedAtom->getHandle(), Temporal::getFromTimeNodeName(nnn->getName().c_str()), TemporalTable::EXACT, timeDomain);

}
