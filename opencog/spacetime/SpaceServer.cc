/*
 * opencog/spacetime/SpaceServer.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
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
#include <string>
#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/TruthValue.h>

#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>

#include <opencog/spacetime/atom_types.h>
#include "SpaceServer.h"
#include "TimeServer.h"

//#define DPRINTF printf
#define DPRINTF(...)
// To simplify debug log output
#define ATOM_AS_STRING(h) (h->to_short_string().c_str())

using namespace opencog;
using namespace opencog::spatial;

SpaceServer::SpaceServer(AtomSpace &_atomspace) :
    atomspace(&_atomspace)
{
    // Default values (should only be used for test purposes)
    agentRadius = 0.25;
    agentHeight = 0;

    curSpaceMapHandle = Handle::UNDEFINED;
    curMap = NULL;
    curEntityRecorder = NULL;

    // connect signals
    removedAtomConnection = _atomspace.addAtomSignal(boost::bind(&SpaceServer::atomAdded, this, _1));
    addedAtomConnection = _atomspace.removeAtomSignal(boost::bind(&SpaceServer::atomRemoved, this, _1));

}

SpaceServer::~SpaceServer()
{
    // disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();

    clear();
}

void SpaceServer::setTimeServer(TimeServer* ts)
{
    timeser = ts;
}

//TODO
void SpaceServer::atomAdded(const Handle& h)
{

}

void SpaceServer::atomRemoved(const AtomPtr& atom)
{
    Type type = atom->getType();
    if (not classserver().isA(type, OBJECT_NODE)) return;

    std::vector<std::string> timeDomains = timeser->getTimeDomains();
    for (auto timeDomain: timeDomains) {
        removeSpaceInfo(atom->getHandle(), curSpaceMapHandle, 0, timeDomain);
    }
}

void SpaceServer::setAgentRadius(unsigned int _radius)
{
    if (agentRadius != _radius) {
        agentRadius = _radius;
        logger().info("SpaceServer - AgentRadius: %d", agentRadius);
    }
}

void SpaceServer::setAgentHeight(unsigned int _height, Handle spaceMapHandle)
{
    auto scenePairItr = scenes.find(spaceMapHandle);
    if ( scenePairItr == scenes.end()) {
        logger().error("SpaceServer::setAgentHeight - Map not found!");
        return;
    }

    SpaceMap& theMap = (scenePairItr->second).spaceMap;

    if (agentHeight != _height) {
        agentHeight = _height;
        theMap.setAgentHeight(_height);
        logger().info("SpaceServer - AgentHeight: %d", agentHeight);
    }

}

const SpaceServer::SpaceMap& SpaceServer::getMap(Handle spaceMapHandle) const
{
    logger().fine("SpaceServer::getMap() for mapHandle = %s",
            spaceMapHandle != Handle::UNDEFINED ?
            ATOM_AS_STRING(spaceMapHandle)
            : "Handle::UNDEFINED");

    auto itr = scenes.find(spaceMapHandle);

    if (itr == scenes.end()) {
        throw opencog::RuntimeException(TRACE_INFO,
                "SpaceServer - Found no SpaceMap associate with handle: '%s'.",
                ATOM_AS_STRING(spaceMapHandle));
    }

    return (itr->second).spaceMap;
}

const EntityRecorder& SpaceServer::getEntityRecorder(Handle spaceMapHandle) const
{
    logger().fine("SpaceServer::getEntityRecorder() for mapHandle = %s",
            spaceMapHandle != Handle::UNDEFINED ?
            ATOM_AS_STRING(spaceMapHandle)
            : "Handle::UNDEFINED");

    auto itr = scenes.find(spaceMapHandle);

    if (itr == scenes.end()) {
        throw opencog::RuntimeException(TRACE_INFO,
                "SpaceServer - Found no EntityRecorder associate with handle: '%s'.",
                ATOM_AS_STRING(spaceMapHandle));
    }

    return (itr->second).entityRecorder;
}


const bool SpaceServer::containsMap(Handle spaceMapHandle) const
{
    return (scenes.find(spaceMapHandle) != scenes.end());
}

const bool SpaceServer::isLatestMapValid() const
{
    return ((curSpaceMapHandle != Handle::UNDEFINED) && (curMap != 0));
}

const SpaceServer::SpaceMap& SpaceServer::getLatestMap() const
{
    return getMap(curSpaceMapHandle);
}

const SpaceServer::EntityRecorder& SpaceServer::getLatestEntityRecorder() const
{
    return getEntityRecorder(curSpaceMapHandle);
}

Handle SpaceServer::getLatestMapHandle() const
{
    return curSpaceMapHandle;
}

void SpaceServer::removeMap(Handle spaceMapHandle)
{
    auto itr = scenes.find(spaceMapHandle);
    if (itr != scenes.end()) {
        scenes.erase(itr);
    }
}

unsigned int SpaceServer::getSpaceMapsSize() const
{
    return scenes.size();
}

void SpaceServer::clear()
{
    scenes.clear();
}

SpaceServer::SpaceMap* SpaceServer::cloneTheLatestSpaceMap() const
{
    return cloneSpaceMap(curSpaceMapHandle);
}

SpaceServer::SpaceMap* SpaceServer::cloneSpaceMap(Handle spaceMapHandle) const
{
    auto itr = scenes.find(spaceMapHandle);
    return (itr == scenes.end())
        ? (itr->second).spaceMap.clone()
        : nullptr;
}

bool SpaceServer::addSpaceInfo(Handle objectNode, Handle spaceMapHandle, bool isSelfObject, bool isAvatarEntity, octime_t timestamp, double objX, double objY, double objZ, const TimeDomain& timeDomain)
{
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().error("SpaceServer::addSpaceInfo - No space map now!");
        return false;
    }

    auto scenePairItr = scenes.find(spaceMapHandle);
    if ( scenePairItr == scenes.end()) {
        logger().error("SpaceServer::removeSpaceInfo - No space map now!");
        return false;
    }

    timeser->addTimeInfo(spaceMapHandle, timestamp, timeDomain);

    // we should distinguish to add a block or other object
    // because when adding a block, maybe cause some change in the terrain and structures

    opencog::spatial::BlockVector pos(objX, objY, objZ);

    if (objectNode->getType() == STRUCTURE_NODE) {
        // it's a block
        SpaceMap& theSpaceMap = (scenePairItr->second).spaceMap;
        theSpaceMap.addSolidUnitBlock(objectNode, pos);
    } else {
        EntityRecorder& entityRecorder = (scenePairItr->second).entityRecorder;
        entityRecorder.addNoneBlockEntity(objectNode, pos, isSelfObject, isAvatarEntity, timestamp);
    }
    return true;
}

Handle SpaceServer::addOrGetSpaceMap(octime_t timestamp, std::string _mapName, double _resolution, const TimeDomain& timeDomain)
{
    Handle spaceMapHandle = atomspace->get_handle(SPACE_MAP_NODE,_mapName);

    // There is not a space map node for this map in the atomspace, so create a new one
    if (spaceMapHandle == Handle::UNDEFINED) {
        spaceMapHandle = atomspace->add_node(SPACE_MAP_NODE,_mapName);
        // spaceMapHandle->setLTI(1);
        timeser->addTimeInfo(spaceMapHandle, timestamp, timeDomain);
        scenes.emplace(std::piecewise_construct, std::make_tuple(spaceMapHandle),
                       std::make_tuple(_mapName, _resolution));
    }

    curSpaceMapHandle = spaceMapHandle;
    return spaceMapHandle;
}

void SpaceServer::removeSpaceInfo(Handle objectNode, Handle spaceMapHandle, octime_t timestamp, const TimeDomain& timeDomain)
{
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().error("SpaceServer::removeSpaceInfo - Undefined SpaceMap!");
        return;
    }

    auto scenePairItr = scenes.find(spaceMapHandle);
    if ( scenePairItr == scenes.end()) {
        logger().error("SpaceServer::removeSpaceInfo - No space map now!");
        return;
    }

    if (timestamp != 0) {
        timeser->addTimeInfo(spaceMapHandle, timestamp, timeDomain);
    }

    if (objectNode->getType() == STRUCTURE_NODE) {
        SpaceMap& theSpaceMap = (scenePairItr->second).spaceMap;
        theSpaceMap.removeSolidUnitBlock(objectNode);
    } else {
        EntityRecorder& entityRecorder = (scenePairItr->second).entityRecorder;
        entityRecorder.removeNoneBlockEntity(objectNode);
    }

    logger().debug("%s(%s)\n", __FUNCTION__, objectNode->getName().c_str());

}

void SpaceServer::mapRemoved(Handle mapId)
{
    // Remove this atom from AtomSpace since its map does not exist anymore
    atomspace->remove_atom(mapId);
}

void SpaceServer::mapPersisted(Handle mapId)
{
    // set LTI to a value that prevents the corresponding atom to be removed
    // from AtomSpace
    // mapId->setLTI(1);
}

std::string SpaceServer::getMapIdString(Handle mapHandle) const
{
    // Currently the mapHandle is of AtTimeLink(TimeNode:"<timestamp>" , ConceptNode:"SpaceMap")
    // So, just get the name of the TimeNode as its string representation
    // return atomspace->get_name(mapHandle->getOutgoingSet()[0])->get_result();

    return mapHandle->getName();
}

// TODO
std::string SpaceServer::mapToString(Handle mapHandle) const
{

    std::stringstream stringMap;
    stringMap.precision(25);
    stringMap << getMapIdString(mapHandle);
    // see localspacemap toString

    return stringMap.str( );
}

// TODO
Handle SpaceServer::mapFromString(const std::string& stringMap)
{
    return Handle::UNDEFINED;
}

Handle SpaceServer::addPropertyPredicate(
        std::string predicateName,
        Handle a,
        Handle b,
        TruthValuePtr tv)
{
    Handle ph = atomspace->add_node(PREDICATE_NODE, predicateName);
    Handle ll =  atomspace->add_link(LIST_LINK, a, b);
    Handle result = atomspace->add_link(EVALUATION_LINK, ph, ll);
    result->setTruthValue(tv);

    return result;
}
