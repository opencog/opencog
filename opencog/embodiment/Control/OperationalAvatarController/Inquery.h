/*
 * Inquery.h
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

#ifndef _OAC_INQUERY_H
#define _OAC_INQUERY_H

#include <vector>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include "PlanningHeaderFiles.h"
#include "Strips.h"

using namespace std;
using namespace opencog;
using namespace opencog::pai;

// some state values need real time inquery, calculation, like the Distance between two moving Entities
// this class offer a series of inquery funciton according to different states

namespace opencog { namespace oac {

class State;
class RuleNode;

class Inquery
{

protected:

    static AtomSpace* atomSpace;
    static SpaceServer::SpaceMap* spaceMap;
    static set<spatial::SPATIAL_RELATION> getSpatialRelations(const vector<ParamValue>& stateOwnerList);

public:

    static void init(AtomSpace* _atomSpace);

    // when we are doing planning, we usually don't use the real spaceMap.
    // Instead, we use a clone of the real spaceMap, which is kinda imaginary spaceMap
    // Call this funciton to assign the imaginary spaceMap here
    static void setSpaceMap(SpaceServer::SpaceMap* _spaceMap);

    // After planning, please reset the spaceMap back to the real one via calling this function
    static void reSetSpaceMap();

    // only apply when getStateOwner is Entity or string
    static Handle getStateOwnerHandle(ParamValue &stateOwnerParamValue);

    // If this is an simple state, which requires no real time calculation.
    // There is EvaluationLink in the atomspace for this state, we can just get its latest value from the atomspace
    // sometimes there is only resutls that has a truth value lower than 0.5, in such case, is_true will be assign false.
    static ParamValue getParamValueFromAtomspace(State &state, bool &is_true);

    // for that kind of States that do not exist in Atomspace, nor in real time system, just simply return UNDEFINED_VALUE
    static ParamValue inqueryUnknowableState(const vector<ParamValue>& stateOwnerList);

    // check if these two are the same, currently only support entity and string
    static ParamValue inqueryIsSame(const vector<ParamValue>& stateOwnerList);

    static ParamValue inqueryDistance(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryExist(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryAtLocation(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsSolid(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsStandable(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryExistPath(const vector<ParamValue>& stateOwnerList);

    // return a vector of all the possible values for grounding a variable in a rule
    // if cannot find proper value, return a empty vector
    static vector<ParamValue> inqueryNearestAccessiblePosition(const vector<ParamValue>& stateOwnerList);
    static vector<ParamValue> inqueryBestAccessiblePosition(const vector<ParamValue>& stateOwnerList);
    static vector<ParamValue> inqueryAdjacentPosition(const vector<ParamValue>& stateOwnerList);
    static vector<ParamValue> inqueryAdjacentAccessPosition(const vector<ParamValue>& stateOwnerList);
    /*static vector<ParamValue> inqueryStandableNearbyAccessablePosition(const vector<ParamValue>& stateOwnerList);*/
    static vector<ParamValue> inqueryUnderPosition(const vector<ParamValue>& stateOwnerList);// get the position just under the input pos



    // inquery the spatial relationships
    // see the definition of SPATIAL_RELATION in Octree3DMapManager.h
    /*enum SPATIAL_RELATION
    {
        LEFT_OF = 0,
        RIGHT_OF,
        ABOVE,
        BELOW,
        BEHIND,
        IN_FRONT_OF,
        BESIDE,
        NEAR,
        FAR_,
        TOUCHING,
        BETWEEN,
        INSIDE,
        OUTSIDE,

        TOTAL_RELATIONS
    };*/
    // relations for two objects themselves
    static ParamValue inqueryIsAbove(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsBeside(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsNear(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsFar(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsTouching(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsInside(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsOutside(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsBelow(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsLeftOf(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsRightOf(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsBehind(const vector<ParamValue>& stateOwnerList);
    static ParamValue inqueryIsInFrontOf(const vector<ParamValue>& stateOwnerList);

    // only use in two position, in 3D block world, one is possible to be able to move from one position to the adjacent position
    // so the adjacent is the 24 neighbours, (the 26 neighbours except the block right above it and under it)
    static ParamValue inqueryIsAdjacent(const vector<ParamValue>& stateOwnerList);

    // relations for 3 objects
    static ParamValue inqueryIsBetween(const vector<ParamValue>& stateOwnerList);

    // to search for all the handles of the Entities meet the given condition from the Atomspace
    static HandleSeq findAllObjectsByGivenCondition(State* state);

    // generate a node for one matching condition for using Pattern Matching, from a state in the precondition list of a RuleNode
    // it can be a const node or a variable node
    // this function is used by generatePMLinkFromAState()
    static HandleSeq generatePMNodeFromeAParamValue(ParamValue& ParamValue, RuleNode* ruleNode);

    // only for grounded paramvalue
    static HandleSeq generateNodeForGroundedParamValue(ParamValue* realValue);

    // generate a link for one matching condition for using Pattern Matching, from a state in the precondition list of a RuleNode
    static Handle generatePMLinkFromAState(State* state, RuleNode* ruleNode);

    // the state indexes vector is the indexes of states in the curUngroundedVariables of this rule node that will be used as conditions of patttern matching query in this function
    static HandleSeq findCandidatesByPatternMatching(RuleNode *ruleNode, vector<int> &stateIndexes, vector<string> &varNames);

    static ParamValue getParamValueFromHandle(string var, Handle& valueH);


};


}}


#endif
