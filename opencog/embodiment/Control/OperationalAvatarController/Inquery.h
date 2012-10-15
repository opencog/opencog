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

#include "OAC.h"
#include "PlanningHeaderFiles.h"

using namespace std;
using namespace opencog;
using namespace opencog::pai;

// some state values need real time inquery, calculation, like the Distance between two moving Entities
// this class offer a series of inquery funciton according to different states

namespace opencog { namespace oac {

class Inquery
{

protected:
    static OAC* oac;
    static set<spatial::SPATIAL_RELATION> getSpatialRelations(vector<StateValue>& stateOwnerList);

public:

    static void init(OAC* _oac);

    static StateValue inqueryDistance(vector<StateValue>& stateOwnerList);
    static StateValue inqueryExist(vector<StateValue>& stateOwnerList);
    static StateValue inqueryEnergy(vector<StateValue>& stateOwnerList);
    static StateValue inqueryAtLocation(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsSolid(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsStandable(vector<StateValue>& stateOwnerList);
    static StateValue inqueryExistPath(vector<StateValue>& stateOwnerList);

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
    static StateValue inqueryIsAbove(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsBeside(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsNear(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsFar(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsTouching(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsInside(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsOutside(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsBelow(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsLeftOf(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsRightOf(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsBehind(vector<StateValue>& stateOwnerList);
    static StateValue inqueryIsInFrontOf(vector<StateValue>& stateOwnerList);

    // only use in two position, in 3D block world, one is possible to be able to move from one position to the adjacent position
    // so the adjacent is the 24 neighbours, (the 26 neighbours except the block right above it and under it)
    static StateValue inqueryIsAdjacent(vector<StateValue>& stateOwnerList);

    // relations for 3 objects
    static StateValue inqueryIsBetween(vector<StateValue> stateOwnerList);


};


}}


#endif
