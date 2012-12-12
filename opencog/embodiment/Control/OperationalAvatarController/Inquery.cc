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

#include "Inquery.h"

using namespace opencog::oac;
using namespace opencog::pai;

AtomSpace* Inquery::atomSpace= 0;
SpaceServer::SpaceMap* Inquery::spaceMap = 0;
OAC* Inquery::oac = 0;


 void Inquery::init(OAC* _oac,AtomSpace* _atomSpace)
 {
    oac = _oac;
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

 StateValue Inquery::getStateValueFromAtomspace( State& state)
 {
     vector<StateValue> stateOwnerList = state.stateOwnerList;
     Entity entity1, entity2, entity3;
     Handle a, b, c = Handle::UNDEFINED;

     entity1 = boost::get<Entity>(stateOwnerList[0]);
     a = AtomSpaceUtil::getEntityHandle(*atomSpace,entity1.id);

     if(stateOwnerList.size() > 0)
     {
         entity2 = boost::get<Entity>(stateOwnerList[1]);
         b = AtomSpaceUtil::getEntityHandle(*atomSpace,entity2.id);
     }
     if(stateOwnerList.size() > 1)
     {
         entity3 = boost::get<Entity>(stateOwnerList[2]);
         c = AtomSpaceUtil::getEntityHandle(*atomSpace,entity3.id);
     }

     Handle evalLink = AtomSpaceUtil::getLatestEvaluationLink(*atomSpace, state.name(), a , b, c);
     if (evalLink == Handle::UNDEFINED)
     {
         logger().error("Inquery::getStateValueFromAtomspace : There is no evaluation link for predicate: "
                  + state.name() );
         return UNDEFINED_VALUE;
     }

     Handle listLink = atomSpace->getOutgoing(evalLink, 1);

     if (listLink== Handle::UNDEFINED)
     {
         logger().error("Inquery::getStateValueFromAtomspace : There is no list link for the Evaluation Link: "
                  + state.name());
         return UNDEFINED_VALUE;
     }

     // So the value node is always after the owners.
     // We need to know how many owners this state has

     // the size of the state owners
     int ownerSize = stateOwnerList.size();

     HandleSeq listLinkOutgoings = atomSpace->getOutgoing(listLink);


     if ( listLinkOutgoings.size() == ownerSize )
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
         if (state.getStateValuleType().getCode() != BOOLEAN_CODE)
         {
             logger().error("Inquery::getStateValueFromAtomspace : There is no value node for this Evaluation Link: "
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
             logger().error("Inquery::getStateValueFromAtomspace : There is no list link for the Evaluation Link: "
                      + state.name());
             return UNDEFINED_VALUE;
         }
         // this kind of state value can only be bool,int,float,string or entity
         switch (state.getStateValuleType().getCode())
         {
         case BOOLEAN_CODE:
         case INT_CODE:
         case FLOAT_CODE:
         case STRING_CODE:

             return (atomSpace->getName(valueHandle));

         case ENTITY_CODE:
             return Entity(atomSpace->getName(valueHandle) ,PAI::getObjectTypeFromHandle(*atomSpace, valueHandle));

         default:
             logger().error("Inquery::getStateValueFromAtomspace : There is more than one value node for the Evaluation Link: "
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
             logger().error("Inquery::getStateValueFromAtomspace :Type error: The value type is fuzzy interval, but there are not 2 number nodes in its listlink , for the Evaluation Link: "
                            + state.name() );
             return UNDEFINED_VALUE;
         }

         switch (state.getStateValuleType().getCode())
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
             logger().error("Inquery::getStateValueFromAtomspace : Type error: There is 2 number nodes for the Evaluation Link: "
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
             logger().error("Inquery::getStateValueFromAtomspace :Type error: The value type is vector or rotation,but there are not 3 number nodes in its listlink , for the Evaluation Link: "
                            + state.name() );
             return UNDEFINED_VALUE;
         }

         double x = atof(atomSpace->getName(valueHandle1).c_str());
         double y = atof(atomSpace->getName(valueHandle2).c_str());
         double z = atof(atomSpace->getName(valueHandle3).c_str());

         switch (state.getStateValuleType().getCode())
         {
         case VECTOR_CODE:
             return Vector(x,y,z);

         case ROTATION_CODE:
             return Rotation(x,y,z);
         default:
             logger().error("Inquery::getStateValueFromAtomspace : Type error: There is 3 number nodes for the Evaluation Link: "
                            + state.name() + ". But it is neighter a Vector nor a Rotation");
             return UNDEFINED_VALUE;
         }
     }
     else
     {
         logger().error("Inquery::getStateValueFromAtomspace :the number of value nodes is invalid for the Evaluation Link: "
                        + state.name() );
         return UNDEFINED_VALUE;
     }

 }

StateValue Inquery::inqueryDistance(const vector<StateValue>& stateOwnerList)
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

StateValue Inquery::inqueryExist(const vector<StateValue>& stateOwnerList)
{
    Entity entity = boost::get<Entity>(stateOwnerList.front());
    // if (! entity)
    //    return "false";
    bool is_exist = spaceMap->containsObject(entity.id);
    return (opencog::toString(is_exist));
}

StateValue Inquery::inqueryEnergy(const vector<StateValue>& stateOwnerList)
{
/*
    Entity entity = boost::get<Entity>(stateOwnerList.front());
    if (! entity)
       return (opencog::toString(0.0));
*/
    double energyValue = oac->getPsiDemandUpdaterAgent()->getDemandValue("Energy");
    return (opencog::toString(energyValue));

}

StateValue Inquery::inqueryAtLocation(const vector<StateValue>& stateOwnerList)
{
     StateValue obj = stateOwnerList.front();

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

StateValue Inquery::inqueryIsSolid(const vector<StateValue>& stateOwnerList)
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

StateValue Inquery::inqueryIsStandable(const vector<StateValue>& stateOwnerList)
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

StateValue Inquery::inqueryExistPath(const vector<StateValue>& stateOwnerList)
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

StateValue Inquery::inqueryIsAdjacent(const vector<StateValue>& stateOwnerList)
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

StateValue Inquery::inqueryIsAbove(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::ABOVE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBeside(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BESIDE) != relations.end())
        return "true";
    else
        return "false";
}
StateValue Inquery::inqueryIsNear(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::NEAR) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsFar(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::FAR_) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsTouching(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::TOUCHING) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsInside(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::INSIDE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsOutside(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::OUTSIDE) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBelow(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BELOW) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsLeftOf(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::LEFT_OF) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsRightOf(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::RIGHT_OF) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsBehind(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::BEHIND) != relations.end())
        return "true";
    else
        return "false";
}

StateValue Inquery::inqueryIsInFrontOf(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> relations = getSpatialRelations(stateOwnerList);
    if (relations.find(spatial::IN_FRONT_OF) != relations.end())
        return "true";
    else
        return "false";
}

set<spatial::SPATIAL_RELATION> Inquery::getSpatialRelations(const vector<StateValue>& stateOwnerList)
{
    set<spatial::SPATIAL_RELATION> empty;
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
