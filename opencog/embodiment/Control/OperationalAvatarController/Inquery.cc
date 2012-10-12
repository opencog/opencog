/*
 * OCPlanner.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * Written by Shujing KE
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

#include "Inquery.h"
#include <set>
#include <opencog/util/oc_assert.h>
#include <opencog/spatial/3DSpaceMap/Entity3D.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/spatial/3DSpaceMap/Octree3DMapManager.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>
#include <opencog/spatial/3DSpaceMap/Pathfinder3D.h>
#include <boost/variant.hpp>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>

using namespace opencog::oac;
using namespace opencog::pai;

 SpaceServer::SpaceMap* Inquery::spaceMap = 0;
 OAC* Inquery::oac = 0;

void Inquery::init(OAC* _oac)
{
    oac = _oac;
    spaceMap = (SpaceServer::SpaceMap*) (&(oac->getAtomSpace()->getSpaceServer().getLatestMap()));
}

StateValue Inquery::inqueryDistance(vector<StateValue>& stateOwnerList)
{
    double d = DOUBLE_MAX;
    StateValue var1 = stateOwnerList.front();
    StateValue var2 = stateOwnerList.back();

    Entity* entity1 = boost::get<Entity>(&var1);
    Entity* entity2 = boost::get<Entity>(&var2);
    Vector* vector2 = boost::get<Vector>(&var2);
    if (entity1 && entity2)
    {
        d = spaceMap->distanceBetween(entity1->id, entity2->id);
    }
    else if (entity1 && vector2)
    {
        d = spaceMap->distanceBetween(entity1->id, SpaceServer::SpaceMapPoint(vector2->x,vector2->y,vector2->z));
    }

    return (opencog::toString(d));
}

StateValue Inquery::inqueryExist(vector<StateValue>& stateOwnerList)
{
    Entity* entity = boost::get<Entity>(& (StateValue)(stateOwnerList.front()));
    if (! entity)
        return "false";
    bool is_exist = spaceMap->containsObject(entity->id);
    return (opencog::toString(is_exist));
}

StateValue Inquery::inqueryEnergy(vector<StateValue>& stateOwnerList)
{
    Entity* entity = boost::get<Entity>( &(StateValue)(stateOwnerList.front()));
    if (! entity)
        return (opencog::toString(0.0));
    double energyValue = oac->getPsiDemandUpdaterAgent()->getDemandValue("Energy");
    return (opencog::toString(energyValue));
}

StateValue Inquery::inqueryAtLocation(vector<StateValue>& stateOwnerList)
{
     StateValue obj = stateOwnerList.front();

     Entity* entity = boost::get<Entity>(&obj);

    Handle h = AtomSpaceUtil::getObjectHandle(oac->getAtomSpaceReference(), entity->id);
    SpaceServer::SpaceMapPoint pos;
    if (h == Handle::UNDEFINED) // not an object, let try if it's an agent
        h = AtomSpaceUtil::getAgentHandle(oac->getAtomSpaceReference(), entity->id);

    if(h == Handle::UNDEFINED)
        return Vector(DOUBLE_MAX,DOUBLE_MAX,DOUBLE_MAX);
    else
         pos = spaceMap->getObjectLocation(h);

    if (pos == SpaceServer::SpaceMapPoint::ZERO)
        return Vector(DOUBLE_MAX,DOUBLE_MAX,DOUBLE_MAX);
    else
        return Vector(pos.x,pos.y,pos.z);
}

StateValue Inquery::inqueryIsSolid(vector<StateValue>& stateOwnerList)
{
    StateValue var = stateOwnerList.back();
    Vector* pos = boost::get<Vector>(&var);
    if (! pos)
        return "false";

    if (spaceMap->checkIsSolid((int)pos->x,(int)pos->y,(int)pos->z))
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsStandable(vector<StateValue>& stateOwnerList)
{
    StateValue var = stateOwnerList.back();
    Vector* pos = boost::get<Vector>(&var);
    if (! pos)
        return "false";

    if (spaceMap->checkStandable((int)pos->x,(int)pos->y,(int)pos->z))
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryExistPath(vector<StateValue>& stateOwnerList)
{
    StateValue var1 = stateOwnerList.front();
    StateValue var2 = stateOwnerList.back();
    spatial::BlockVector pos1,pos2;

    Entity* entity1 = boost::get<Entity>(&var1);
    if (entity1)
        pos1 = spaceMap->getObjectLocation(entity1->id);
    else
    {
        Vector* v1 = boost::get<Vector>(&var1);
        pos1 = SpaceServer::SpaceMapPoint(v1->x,v1->y,v1->z);
    }

    Entity* entity2 = boost::get<Entity>(&var2);
    if (entity2)
        pos2 = spaceMap->getObjectLocation(entity2->id);
    else
    {
        Vector* v2 = boost::get<Vector>(&var2);
        pos2 = SpaceServer::SpaceMapPoint(v2->x,v2->y,v2->z);
    }

    if (SpaceServer::SpaceMap::isTwoPositionsAdjacent(pos1, pos2))
    {
        if (spatial::Pathfinder3D::checkNeighbourAccessable(spaceMap, pos1, pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z))
            return "true";
        else
            return "false";
    }

    vector<spatial::BlockVector> path;
    if (spatial::Pathfinder3D::AStar3DPathFinder(spaceMap, pos1, pos2, path))
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsAdjacent(vector<StateValue>& stateOwnerList)
{
    StateValue var1 = stateOwnerList.front();
    StateValue var2 = stateOwnerList.back();
    Vector* v1 = boost::get<Vector>(&var1);
    Vector* v2 = boost::get<Vector>(&var2);

    if ((!v1)||(!v2))
        return "false";

    SpaceServer::SpaceMapPoint pos1(v1->x,v1->y,v1->z);
    SpaceServer::SpaceMapPoint pos2(v2->x,v2->y,v2->z);

    if (SpaceServer::SpaceMap::isTwoPositionsAdjacent(pos1, pos2))
        return "true";
    else
        return "false";

}

StateValue Inquery::inqueryIsAbove(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::ABOVE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBeside(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BESIDE) != relations.end())
        return "true";
    else
        return "false";
}
StateValue Inquery::inqueryIsNear(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::NEAR) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsFar(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::FAR_) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsTouching(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::TOUCHING) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsInside(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::INSIDE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsOutside(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::OUTSIDE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBelow(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BELOW) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsLeftOf(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::LEFT_OF) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsRightOf(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::RIGHT_OF) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBehind(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BEHIND) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsInFrontOf(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::IN_FRONT_OF) != relations.end())
        return "true";
    else
        return "false";
}

set<spatial::SPATIAL_RELATION> Inquery::getSpatialRelations(vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> empty;
    Entity* entity1 = boost::get<Entity>( &((StateValue)(stateOwnerList.front())));
    Entity* entity2 = boost::get<Entity>( &((StateValue)(stateOwnerList[1])));
    Entity* entity3 = boost::get<Entity>( &((StateValue)(stateOwnerList[2])));
    if ((entity1 == 0)||(entity2 == 0))
        return empty;

    string entity3ID = "";
    if (stateOwnerList.size() == 3)
     {
         Entity* entity3 = boost::get<Entity>( &((StateValue)(stateOwnerList.back())));
         entity3ID = entity3->id;

     }

    return  spaceMap->computeSpatialRelations(entity1->id,entity2->id,entity3->id);
}
