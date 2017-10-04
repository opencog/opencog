/*
 * opencog/spatial/3DSpaceMap/Pathfinder3D.h
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Shujing Ke
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

#ifndef _SPATIAL_PATHFINDER3D_H
#define _SPATIAL_PATHFINDER3D_H

#include <vector>
#include <opencog/atomspace/AtomSpace.h>
#include "Block3DMapUtil.h"


using namespace std;

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */

    namespace spatial
    {
        class AtomOcTree;

        class Pathfinder3D
        {
            public:
                // When getNearestPos is true,return the nearestPos as well, which would possibably useful when
                // it cannot find a path,at least it find the nearest location to the target;
                // The bestPos is calculated by the A* heuristics which consider the cost of moving and the distance
                //  to the target, heuristic = (target - pos)*1.41421356f + (begin - pos)

                static bool AStar3DPathFinder(AtomSpace* atomSpace, octomap::AtomOcTree<Handle>* mapManager,
                        const BlockVector& begin, const BlockVector& target,
                        vector<BlockVector>& path, BlockVector& nearestPos,
                        BlockVector& bestPos, bool getNearestPos = false, 
                        bool getBestPos = false, bool tryOptimal = false);

                static double calculateCostByDistance(const BlockVector& begin,const BlockVector& target,
                        const BlockVector& pos,float &nearestDis,BlockVector& nearestPos,
                        float& bestHeuristic, BlockVector& bestPos);

                static bool checkNeighbourAccessable(octomap::AtomOcTree<Handle> *mapManager, BlockVector& lastPos, int i, int j, int k);
        };
    }
    /** @}*/
}







#endif // _SPATIAL_PATHFINDER3D_H
