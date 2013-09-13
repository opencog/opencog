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

#include <set>
#include <boost/variant.hpp>

#include <opencog/util/oc_assert.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/spacetime/SpaceServer.h>

#include <opencog/spatial/3DSpaceMap/Entity3D.h>
#include <opencog/spatial/3DSpaceMap/Octree3DMapManager.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>
#include <opencog/spatial/3DSpaceMap/Pathfinder3D.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParamType.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/server/BaseServer.h>
#include <opencog/query/PatternMatch.h>
#include <opencog/query/PatternMatchEngine.h>
#include <opencog/util/StringManipulator.h>

#include <opencog/atomspace/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#include "Inquery.h"
#include "OCPlanner.h"

using namespace opencog::oac;
using namespace opencog::pai;

AtomSpace* Inquery::atomSpace= 0;
SpaceServer::SpaceMap* Inquery::spaceMap = 0;


 void Inquery::init(AtomSpace* _atomSpace)
 {
    atomSpace = _atomSpace;
    spaceMap = &(spaceServer().getLatestMap());
 }

 void Inquery::setSpaceMap(SpaceServer::SpaceMap *_spaceMap)
 {
     spaceMap = _spaceMap;
 }

 void Inquery::reSetSpaceMap()
 {
     spaceMap = &(spaceServer().getLatestMap());
 }

 ParamValue Inquery::getParamValueFromAtomspace( State& state)
 {
     vector<ParamValue> stateOwnerList = state.stateOwnerList;
     Entity entity1, entity2, entity3;
     Handle a, b, c = Handle::UNDEFINED;

     entity1 = boost::get<Entity>(stateOwnerList[0]);
     a = AtomSpaceUtil::getEntityHandle(*atomSpace,entity1.id);

     if(stateOwnerList.size() > 1)
     {
         entity2 = boost::get<Entity>(stateOwnerList[1]);
         b = AtomSpaceUtil::getEntityHandle(*atomSpace,entity2.id);
     }
     if(stateOwnerList.size() > 2)
     {
         entity3 = boost::get<Entity>(stateOwnerList[2]);
         c = AtomSpaceUtil::getEntityHandle(*atomSpace,entity3.id);
     }

     Handle evalLink = AtomSpaceUtil::getLatestEvaluationLink(*atomSpace, state.name(), a , b, c);
     if (evalLink == Handle::UNDEFINED)
     {
         logger().error("Inquery::getParamValueFromAtomspace : There is no evaluation link for predicate: "
                  + state.name() );
         return UNDEFINED_VALUE;
     }

     Handle listLink = atomSpace->getOutgoing(evalLink, 1);

     if (listLink== Handle::UNDEFINED)
     {
         logger().error("Inquery::getParamValueFromAtomspace : There is no list link for the Evaluation Link: "
                  + state.name());
         return UNDEFINED_VALUE;
     }

     // So the value node is always after the owners.
     // We need to know how many owners this state has

     // the size of the state owners
     int ownerSize = stateOwnerList.size();

     HandleSeq listLinkOutgoings = atomSpace->getOutgoing(listLink);


     if ( listLinkOutgoings.size() == (std::size_t)ownerSize )
     {   // some evalLink without a value node, e.g.:
         /*    It means these two objects are far away from each other, we'll get its value from its truthvalue
        (EvaluationLink (stv 1 0.0012484394) (av -14 1 0)
           (PredicateNode "far" (av -21 1 0))
           (ListLink (av -12 0 0)
              (ObjectNode "id_-1522462" (av -12 1 0))
              (ObjectNode "id_5640" (av -20 0 0))
           )
        )
        */
         // then the state value type should be boolean
         if (state.getActionParamType().getCode() != BOOLEAN_CODE)
         {
             logger().error("Inquery::getParamValueFromAtomspace : There is no value node for this Evaluation Link: "
                      + state.name());
             return UNDEFINED_VALUE;
         }

         if (atomSpace->getMean(evalLink) > 0.5)
             return "true";
         else
             return "false";
     }
     else if ( listLinkOutgoings.size() - ownerSize == 1)
     {
         //         some state only has one value node, which is the most common state,e.g.:
         //         (EvaluationLink (stv 1 0.0012484394) (av -23 1 0)
         //            (PredicateNode "material" (av -23 1 0))
         //            (ListLink (av -23 0 0)
         //               (StructureNode "id_CHUNK_2_2_0_BLOCK_15_6_104" (av -23 0 0))
         //               (ConceptNode "Stone" (av -23 0 0))
         //            )
         //         )
         Handle valueHandle = atomSpace->getOutgoing(listLink, ownerSize);// the ownerSize is just the index of the value node

         if ( valueHandle == Handle::UNDEFINED )
         {
             logger().error("Inquery::getParamValueFromAtomspace : There is no list link for the Evaluation Link: "
                      + state.name());
             return UNDEFINED_VALUE;
         }
         // this kind of state value can only be bool,int,float,string or entity
         switch (state.getActionParamType().getCode())
         {
         case BOOLEAN_CODE:
         case INT_CODE:
         case FLOAT_CODE:
         case STRING_CODE:

             return (atomSpace->getName(valueHandle));

         case ENTITY_CODE:
             return Entity(atomSpace->getName(valueHandle) ,PAI::getObjectTypeFromHandle(*atomSpace, valueHandle));

         default:
             logger().error("Inquery::getParamValueFromAtomspace : There is more than one value node for the Evaluation Link: "
                      + state.name());
             return UNDEFINED_VALUE;

         }

     }
     else if (listLinkOutgoings.size() - ownerSize == 2)
     {
         // when this state value is fuzzy number interval, there will be 2 number nodes for the value
         Handle valueHandle1 = atomSpace->getOutgoing(listLink, ownerSize);// the ownerSize is just the index of the value node
         Handle valueHandle2 = atomSpace->getOutgoing(listLink, ownerSize + 1);

         if ( (valueHandle1 == Handle::UNDEFINED) || (valueHandle2 == Handle::UNDEFINED) )
         {
             logger().error("Inquery::getParamValueFromAtomspace :Type error: The value type is fuzzy interval, but there are not 2 number nodes in its listlink , for the Evaluation Link: "
                            + state.name() );
             return UNDEFINED_VALUE;
         }

         switch (state.getActionParamType().getCode())
         {
         case FUZZY_INTERVAL_INT_CODE:
         {
             int lowInt = atoi(atomSpace->getName(valueHandle1).c_str());
             int highInt = atoi(atomSpace->getName(valueHandle2).c_str());
             return FuzzyIntervalInt(lowInt,highInt);
         }

         case FUZZY_INTERVAL_FLOAT_CODE:
         {
             float lowf = atof(atomSpace->getName(valueHandle1).c_str());
             float highf = atof(atomSpace->getName(valueHandle2).c_str());
             return FuzzyIntervalFloat(lowf,highf);
         }

         default:
             logger().error("Inquery::getParamValueFromAtomspace : Type error: There is 2 number nodes for the Evaluation Link: "
                            + state.name() + ". But it is neighter a fuzzyIntervalInt nor a fuzzyIntervalFLoat");
             return UNDEFINED_VALUE;
         }
     }
     else if(listLinkOutgoings.size() - ownerSize == 3)
     {// when it is a vector, it will have 3 number nodes, e.g.:
         //        the link and nodes are like this:
         //        (EvaluationLink (av -23 0 0) // this the "valueH"
         //           (PredicateNode "AGISIM_position" (av -23 1 0))
         //           (ListLink (av -23 0 0)
         //              (StructureNode "id_CHUNK_2_2_0_BLOCK_6_6_104" (av -23 0 0))
         //              (NumberNode "54.5" (av -23 0 0))
         //              (NumberNode "54.5" (av -23 0 0))
         //              (NumberNode "104.5" (av -23 0 0))
         //           )
         //        )
         Handle valueHandle1 = atomSpace->getOutgoing(listLink, ownerSize);// the ownerSize is just the index of the value node
         Handle valueHandle2 = atomSpace->getOutgoing(listLink, ownerSize + 1);
         Handle valueHandle3 = atomSpace->getOutgoing(listLink, ownerSize + 2);

         if ( (valueHandle1 == Handle::UNDEFINED) || (valueHandle2 == Handle::UNDEFINED) || (valueHandle3 == Handle::UNDEFINED) )
         {
             logger().error("Inquery::getParamValueFromAtomspace :Type error: The value type is vector or rotation,but there are not 3 number nodes in its listlink , for the Evaluation Link: "
                            + state.name() );
             return UNDEFINED_VALUE;
         }

         double x = atof(atomSpace->getName(valueHandle1).c_str());
         double y = atof(atomSpace->getName(valueHandle2).c_str());
         double z = atof(atomSpace->getName(valueHandle3).c_str());

         switch (state.getActionParamType().getCode())
         {
         case VECTOR_CODE:
             return Vector(x,y,z);

         case ROTATION_CODE:
             return Rotation(x,y,z);
         default:
             logger().error("Inquery::getParamValueFromAtomspace : Type error: There is 3 number nodes for the Evaluation Link: "
                            + state.name() + ". But it is neighter a Vector nor a Rotation");
             return UNDEFINED_VALUE;
         }
     }
     else
     {
         logger().error("Inquery::getParamValueFromAtomspace :the number of value nodes is invalid for the Evaluation Link: "
                        + state.name() );
         return UNDEFINED_VALUE;
     }

}

ParamValue Inquery::getParamValueFromHandle(string var, Handle& valueH)
{
    switch (opencog::oac::GetVariableType(var))
    {
        case ENTITY_CODE:
        {
            return Entity(atomSpace->getName(valueH) ,PAI::getObjectTypeFromHandle(*atomSpace, valueH));
        }
        case BOOLEAN_CODE:
        {
            string strVar = atomSpace->getName(valueH);
            if ((strVar == "true")||(strVar == "True"))
                return opencog::oac::SV_TRUE;
            else
                return opencog::oac::SV_FALSE;
        }
        case INT_CODE:
        case FLOAT_CODE:
        case STRING_CODE:
        {
            return (atomSpace->getName(valueH));
        }
        case VECTOR_CODE:
        {
            Handle valueHandle1 = atomSpace->getOutgoing(valueH, 0);// the ownerSize is just the index of the value node
            Handle valueHandle2 = atomSpace->getOutgoing(valueH, 1);
            Handle valueHandle3 = atomSpace->getOutgoing(valueH, 2);

            if ( (valueHandle1 == Handle::UNDEFINED) || (valueHandle2 == Handle::UNDEFINED) || (valueHandle3 == Handle::UNDEFINED) )
            {
                logger().error("Inquery::getParamValueFromHandle :Type error: The value type is vector or rotation,but there are not 3 number nodes in its listlink ");

                return UNDEFINED_VALUE;
            }

            double x = atof(atomSpace->getName(valueHandle1).c_str());
            double y = atof(atomSpace->getName(valueHandle2).c_str());
            double z = atof(atomSpace->getName(valueHandle3).c_str());

            return Vector(x,y,z);
        }
        default:
        {
            return UNDEFINED_VALUE;
        }
    }
}

ParamValue Inquery::inqueryDistance(const vector<ParamValue>& stateOwnerList)
{
    double d = DOUBLE_MAX;
    ParamValue var1 = stateOwnerList.front();
    ParamValue var2 = stateOwnerList.back();

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

ParamValue Inquery::inqueryExist(const vector<ParamValue>& stateOwnerList)
{
    Entity entity = boost::get<Entity>(stateOwnerList.front());
    // if (! entity)
    //    return "false";
    bool is_exist = spaceMap->containsObject(entity.id);
    if (is_exist)
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryAtLocation(const vector<ParamValue>& stateOwnerList)
{
     ParamValue obj = stateOwnerList.front();

     Entity* entity = boost::get<Entity>(&obj);

    Handle h = AtomSpaceUtil::getObjectHandle(*atomSpace, entity->id);
    SpaceServer::SpaceMapPoint pos;
    if (h == Handle::UNDEFINED) // not an object, let try if it's an agent
        h = AtomSpaceUtil::getAgentHandle(*atomSpace, entity->id);

    if(h == Handle::UNDEFINED)
        return Vector(DOUBLE_MAX,DOUBLE_MAX,DOUBLE_MAX);
    else
         pos = spaceMap->getObjectLocation(h);

    if (pos == SpaceServer::SpaceMapPoint::ZERO)
        return Vector(DOUBLE_MAX,DOUBLE_MAX,DOUBLE_MAX);
    else
        return Vector(pos.x,pos.y,pos.z);
}

ParamValue Inquery::inqueryIsSolid(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var = stateOwnerList.back();
    Vector* pos = boost::get<Vector>(&var);
    if (! pos)
        return "false";

    if (spaceMap->checkIsSolid((int)pos->x,(int)pos->y,(int)pos->z))
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsStandable(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var = stateOwnerList.back();
    Vector* pos = boost::get<Vector>(&var);
    if (! pos)
        return "false";

    if (spaceMap->checkStandable((int)pos->x,(int)pos->y,(int)pos->z))
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryExistPath(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var1 = stateOwnerList.front();
    ParamValue var2 = stateOwnerList.back();
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
    spatial::BlockVector nearestPos;
    if (spatial::Pathfinder3D::AStar3DPathFinder(spaceMap, pos1, pos2, path, nearestPos))
        return "true";
    else
        return "false";
}

vector<ParamValue> Inquery::inqueryNearestAccessiblePosition(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var1 = stateOwnerList.front();
    ParamValue var2 = stateOwnerList.back();
    spatial::BlockVector pos1,pos2;
    vector<ParamValue> values;

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

    spatial::BlockVector nearestPos;
    vector<spatial::BlockVector> path;
    spatial::Pathfinder3D::AStar3DPathFinder(spaceMap, pos1, pos2, path, nearestPos);

    if (nearestPos != pos1)
    {
        values.push_back(Vector(nearestPos.x,nearestPos.y,nearestPos.z));

    }

    return values;
}

vector<ParamValue> Inquery::inqueryAdjacentPosition(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var1 = stateOwnerList.front();

    spatial::BlockVector pos1;
    vector<ParamValue> values;

    Entity* entity1 = boost::get<Entity>(&var1);
    if (entity1)
        pos1 = spaceMap->getObjectLocation(entity1->id);
    else
    {
        Vector* v1 = boost::get<Vector>(&var1);
        pos1 = SpaceServer::SpaceMapPoint(v1->x,v1->y,v1->z);
    }

    // return 24 adjacent neighbour locations (26 neighbours except the pos above and below)
    int x,y,z;
    for (x = -1; x <= 1; x ++)
        for (y = -1; x <= 1; x ++)
            for (z = -1; z <= 1; z ++)
            {
                if ( (x == 0) && (y == 0))
                    continue;

                values.push_back(Vector(pos1.x + x,pos1.y + y,pos1.z + z));
            }

    return values;
}

vector<ParamValue> Inquery::inqueryStandableNearbyAccessablePosition(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var1 = stateOwnerList.front();
    ParamValue var2 = stateOwnerList[1];

    spatial::BlockVector pos1, pos2;
    vector<ParamValue> values;

    Entity* entity1 = boost::get<Entity>(&var1);

    pos1 = spaceMap->getObjectLocation(entity1->id);


    Vector* v1 = boost::get<Vector>(&var2);
    pos2 = SpaceServer::SpaceMapPoint(v1->x,v1->y,v1->z);


    // return 24 adjacent neighbour locations (26 neighbours except the pos above and below)
    int x,y,z;
    for (x = -1; x <= 1; x ++)
        for (y = -1; x <= 1; x ++)
            for (z = -1; z <= 1; z ++)
            {
                if ( (x == 0) && (y == 0))
                {
                    continue;

                    SpaceServer::SpaceMapPoint curPos(pos2.x + x,pos2.y + y,pos2.z + z);

                    // check if it's standable
                    if (! spaceMap->checkStandable(curPos))
                        continue;
                    // check if there is a path from the avatar to this position
                    if (SpaceServer::SpaceMap::isTwoPositionsAdjacent(pos1, pos2))
                    {
                        if (spatial::Pathfinder3D::checkNeighbourAccessable(spaceMap, pos1, pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z))
                        {
                            values.push_back(Vector(curPos.x,curPos.y,curPos.z));
                        }

                        continue;
                    }

                    vector<spatial::BlockVector> path;
                    spatial::BlockVector nearestPos;
                    if (spatial::Pathfinder3D::AStar3DPathFinder(spaceMap, pos1, pos2, path, nearestPos))
                        values.push_back(Vector(curPos.x,curPos.y,curPos.z));

                }
            }

    return values;
}

ParamValue Inquery::inqueryIsAdjacent(const vector<ParamValue>& stateOwnerList)
{
    ParamValue var1 = stateOwnerList.front();
    ParamValue var2 = stateOwnerList.back();
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

ParamValue Inquery::inqueryIsAbove(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::ABOVE) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsBeside(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BESIDE) != relations.end())
        return "true";
    else
        return "false";
}
ParamValue Inquery::inqueryIsNear(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::NEAR) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsFar(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::FAR_) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsTouching(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::TOUCHING) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsInside(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::INSIDE) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsOutside(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::OUTSIDE) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsBelow(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BELOW) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsLeftOf(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::LEFT_OF) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsRightOf(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::RIGHT_OF) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsBehind(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BEHIND) != relations.end())
        return "true";
    else
        return "false";
}

ParamValue Inquery::inqueryIsInFrontOf(const vector<ParamValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::IN_FRONT_OF) != relations.end())
        return "true";
    else
        return "false";
}

set<spatial::SPATIAL_RELATION> Inquery::getSpatialRelations(const vector<ParamValue>& stateOwnerList)
{
//    set<spatial::SPATIAL_RELATION> empty;
    Entity entity1 = boost::get<Entity>( stateOwnerList.front());
    Entity entity2 = boost::get<Entity>( stateOwnerList[1]);
    Entity entity3 = boost::get<Entity>( stateOwnerList[2]);
/*
    if ((entity1 == 0)||(entity2 == 0))
        return empty;
*/

    string entity3ID = "";
    if (stateOwnerList.size() == 3)
     {
         Entity entity3 = boost::get<Entity>( stateOwnerList.back());
         entity3ID = entity3.id;
     }

    return  spaceMap->computeSpatialRelations(entity1.id, entity2.id, entity3.id);
}

// find atoms by given condition, by patern mactcher.
HandleSeq Inquery::findAllObjectsByGivenCondition(State* state)
{
    HandleSeq results;

    Handle inhSecondOutgoing;
    HandleSeq evalNonFirstOutgoings;

    switch (state->getActionParamType().getCode())
    {
        case ENTITY_CODE:
        {
            const Entity& entity = state->stateVariable->getEntityValue();
            inhSecondOutgoing = AtomSpaceUtil::getEntityHandle(*atomSpace,entity.id);
            if (inhSecondOutgoing == Handle::UNDEFINED)
            {
                logger().warn("Inquery::findAllObjectsByGivenCondition : There is no Entity Node for this state value: "
                              + state->name());
                return results; // return empty result
            }
            evalNonFirstOutgoings.push_back(inhSecondOutgoing);
            break;
        }
        case INT_CODE:
        case FLOAT_CODE:
        {
            const string& value = state->stateVariable->getStringValue();
            inhSecondOutgoing = AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE,value.c_str());
            evalNonFirstOutgoings.push_back(inhSecondOutgoing);
            break;
        }
        case STRING_CODE:
        {
            const string& value = state->stateVariable->getStringValue();
            inhSecondOutgoing = AtomSpaceUtil::addNode(*atomSpace,CONCEPT_NODE,value.c_str());
            evalNonFirstOutgoings.push_back(inhSecondOutgoing);
            break;
        }
        case VECTOR_CODE:
        {
            const Vector& vector = state->stateVariable->getVectorValue();
            // for vector and rotation, the inheritancelink do not apply
            // so only for evaluationlink
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE,opencog::toString(vector.x).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE,opencog::toString(vector.y).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE,opencog::toString(vector.z).c_str()));
            break;
        }
        case ROTATION_CODE:
        {
            const Rotation & value= state->stateVariable->getRotationValue();
            // for vector, rotation and fuzzy values, the inheritancelink do not apply
            // so only for evaluationlink
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.pitch).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.roll).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.yaw).c_str()));
            break;
        }
        case FUZZY_INTERVAL_INT_CODE:
        {
            const FuzzyIntervalInt& value = state->stateVariable->getFuzzyIntervalIntValue();
            // for vector, rotation and fuzzy values, the inheritancelink do not apply
            // so only for evaluationlink
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.bound_low).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.bound_high).c_str()));
            break;
        }
        case FUZZY_INTERVAL_FLOAT_CODE:
        {
            const FuzzyIntervalFloat& value = state->stateVariable->getFuzzyIntervalFloatValue();
            // for vector, rotation and fuzzy values, the inheritancelink do not apply
            // so only for evaluationlink
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.bound_low).c_str()));
            evalNonFirstOutgoings.push_back(AtomSpaceUtil::addNode(*atomSpace, NUMBER_NODE, opencog::toString(value.bound_high).c_str()));
            break;
        }
        default:
            // TODO: TNick: is this the right way of dealing with other codes?
            // BOOLEAN_CODE
            // NUMBER_OF_ACTION_PARAM_TYPES
		    break;
    }

    // first search from EvaluationLinks
    if (evalNonFirstOutgoings.size() > 0)
        results = AtomSpaceUtil::getNodesByEvaluationLink(*atomSpace,state->name(),evalNonFirstOutgoings);

    // if cannot find any result, search from InheritanceLink
    if ((results.size() == 0) && (inhSecondOutgoing != Handle::UNDEFINED))
        results = AtomSpaceUtil::getNodesByInheritanceLink(*atomSpace, inhSecondOutgoing);

    return results;
}



HandleSeq Inquery::generatePMNodeFromeAParamValue(ParamValue& paramValue, RuleNode* ruleNode)
{
    HandleSeq results;

    ParamValue* realValue;

    if (! Rule::isParamValueUnGrounded(paramValue))
    {
        // this stateOwner is const, just add it
        realValue = &paramValue;
    }
    else
    {
        // this stateOwner is a variable
        // look for the value of this variable in the current grounding parameter map

        ParamGroundedMapInARule::iterator paramMapIt = ruleNode->currentBindingsFromForwardState.find(ActionParameter::ParamValueToString(paramValue));
        if (paramMapIt != ruleNode->currentBindingsFromForwardState.end())
        {
            // found it in the current groundings, so just add it as a const
            realValue = &(paramMapIt->second);
        }
        else
        {
            // it has not been grounded, so add it as a variable node
            results.push_back(AtomSpaceUtil::addNode(*atomSpace,VARIABLE_NODE, (ActionParameter::ParamValueToString(paramValue)).c_str()));
            return results;
        }
    }

    string* str = boost::get<string>(realValue);
    Entity* entity ;
    Vector* vector;
    Rotation* rot;
    FuzzyIntervalInt* fuzzyInt;
    FuzzyIntervalFloat* fuzzyFloat;

    if( str)
    {

        if (StringManipulator::isNumber(*str))
        {
            results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, str->c_str()));
        }
        else
        {
            results.push_back(AtomSpaceUtil::addNode(*atomSpace,CONCEPT_NODE, str->c_str()));
        }

    }
    else if ( (entity = boost::get<Entity>(realValue)) )
    {
        Handle entityHandle = AtomSpaceUtil::getEntityHandle(*atomSpace,entity->id);
        OC_ASSERT((entityHandle != Handle::UNDEFINED),
                  "OCPlanner::generatePMNodeFromeAParamValue: cannot find the handle for this entity : %s is invalid.\n",
                  ActionParameter::ParamValueToString(*realValue).c_str());
        results.push_back(entityHandle);
    }
    else if ( (vector = boost::get<Vector>(realValue)) )
    {
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(vector->x).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(vector->y).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(vector->z).c_str()));
    }
    else if ( (rot = boost::get<Rotation>(realValue)) )
    {
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(rot->pitch).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(rot->roll).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(rot->yaw).c_str()));
    }
    else if ( (fuzzyInt = boost::get<FuzzyIntervalInt>(realValue)) )
    {
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(fuzzyInt->bound_low).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(fuzzyInt->bound_high).c_str()));
    }
    else if ( (fuzzyFloat = boost::get<FuzzyIntervalFloat>(realValue)) )
    {
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(fuzzyFloat->bound_low).c_str()));
        results.push_back(AtomSpaceUtil::addNode(*atomSpace,NUMBER_NODE, opencog::toString(fuzzyFloat->bound_high).c_str()));
    }

    return results;

}

// return an EvaluationLink with variableNodes for using Pattern Matching
Handle Inquery::generatePMLinkFromAState(State* state, RuleNode* ruleNode)
{
    // Create evaluationlink used by pattern matcher
    Handle predicateNode = AtomSpaceUtil::addNode(*atomSpace,PREDICATE_NODE, state->name().c_str());

    HandleSeq predicateListLinkOutgoings;

    // add all the stateOwners
    for (vector<ParamValue>::iterator ownerIt = state->stateOwnerList.begin(); ownerIt != state->stateOwnerList.end(); ++ ownerIt)
    {
        HandleSeq handles = generatePMNodeFromeAParamValue(*ownerIt,ruleNode);
        OC_ASSERT((handles.size() != 0),
                  "OCPlanner::generatePMLinkFromAState: cannot generate handle for this state owner value for state: %s is invalid.\n",
                  state->name().c_str());
        predicateListLinkOutgoings.insert(predicateListLinkOutgoings.end(), handles.begin(),handles.end());

    }

    // add the state value
    HandleSeq handles = generatePMNodeFromeAParamValue(state->stateVariable->getValue(),ruleNode);
    predicateListLinkOutgoings.insert(predicateListLinkOutgoings.end(), handles.begin(),handles.end());

    Handle predicateListLink = AtomSpaceUtil::addLink(*atomSpace,LIST_LINK, predicateListLinkOutgoings);

    HandleSeq evalLinkOutgoings;
    evalLinkOutgoings.push_back(predicateNode);
    evalLinkOutgoings.push_back(predicateListLink);
    Handle hEvalLink = AtomSpaceUtil::addLink(*atomSpace,EVALUATION_LINK, evalLinkOutgoings);

    return hEvalLink;

}

HandleSeq Inquery::findCandidatesByPatternMatching(RuleNode *ruleNode, vector<int> &stateIndexes, vector<string>& varNames)
{
    HandleSeq variableNodes,andLinkOutgoings, implicationLinkOutgoings, bindLinkOutgoings;
    vector<string> allVariables;

    if (stateIndexes.size() == 1) // only contains one condition
    {
        int index = stateIndexes[0];
        list<UngroundedVariablesInAState>::iterator it = ruleNode->curUngroundedVariables.begin();
        for(int x = 0; x < index; ++x)
             ++ it;

        UngroundedVariablesInAState& record = (UngroundedVariablesInAState&)(*it);
        std::copy(record.vars.begin(),record.vars.end(),std::back_inserter(allVariables));

        implicationLinkOutgoings.push_back(record.PMLink);
    }
    else
    {
        // contains mutiple conditions, so add them one by one
        vector<string> _allVariables;
        for(int i = 0; (std::size_t)i < stateIndexes.size() ; ++ i)
        {
            int index = stateIndexes[i];
            list<UngroundedVariablesInAState>::iterator it = ruleNode->curUngroundedVariables.begin();
            for(int x = 0; x < index; ++x)
                 ++ it;

            UngroundedVariablesInAState& record = (UngroundedVariablesInAState&)(*it);
            std::copy(record.vars.begin(),record.vars.end(),std::back_inserter(_allVariables));
            andLinkOutgoings.push_back(record.PMLink);
        }

        // remove the repeated elements
        std::unique_copy(_allVariables.begin(),_allVariables.end(),std::back_inserter(allVariables));
        Handle hAndLink = AtomSpaceUtil::addLink(*atomSpace,AND_LINK,andLinkOutgoings);

        implicationLinkOutgoings.push_back(hAndLink);

    }

    // add variable atoms
    vector<string>::iterator itor = allVariables.begin();
    for(;itor != allVariables.end(); ++ itor)
    {
        variableNodes.push_back(AtomSpaceUtil::addNode(*atomSpace,VARIABLE_NODE,(*itor).c_str()));
        varNames.push_back((*itor).c_str());
    }

    Handle hVariablesListLink = AtomSpaceUtil::addLink(*atomSpace,LIST_LINK,variableNodes);

    implicationLinkOutgoings.push_back(hVariablesListLink);

    Handle hImplicationLink = AtomSpaceUtil::addLink(*atomSpace,IMPLICATION_LINK, implicationLinkOutgoings);

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = AtomSpaceUtil::addLink(*atomSpace,BIND_LINK, bindLinkOutgoings);

//    std::cout<<"Debug: Inquery variables from the Atomspace: " << std::endl
//            << atomSpace->atomAsString(hBindLink).c_str() <<std::endl;

    // Run pattern matcher
    PatternMatch pm;
    pm.set_atomspace(atomSpace);

    Handle hResultListLink = pm.bindlink(hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink
    HandleSeq resultSet = atomSpace->getOutgoing(hResultListLink);
    atomSpace->removeAtom(hResultListLink);

    return resultSet;

}
