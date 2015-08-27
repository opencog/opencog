/*
 * opencog/spatial/3DSpaceMap/Octree3DMapManager.h
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

#ifndef _SPATIAL_NEW_OCTREE3DMAPMANAGER_H
#define _SPATIAL_NEW_OCTREE3DMAPMANAGER_H


#include <map>
#include <set>
#include <vector>

#ifdef HAVE_ZMQ
#include <lib/zmq/zmq.hpp>
#endif

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

#include "Block3DMapUtil.h"
#include "OctomapOcTree.h"

using namespace std;





namespace opencog
{
    /** \addtogroup grp_spatial
     *  @{
     */
	
    //Comment on 20150824 by YiShan
    //For now the 3DSpaceMap using Octomap as Octree to save block
    //So we provide the probabilistic feature for each function about block
    //You can directly use the interface without probability 
    //as if the occupancy is binary.
    //The library will control the occupancy probability automatically
    //But you can use the interface with probability
    //to control the occupancy of block.
    //You can also set the occupancy threshold to 
    //change the judgement of block occupancy.

    //Also, for the generic use of SpaceMap, we abandon the old
    //Block3D/Entity3D/BlockEntity class.
    //Since in different use case we want to save different infos.
    //It's better to save/query all the infos in AtomSpace.
    //And the SpaceMap should be used for indexing the block handle.
	
    //For now there are some parts unfinished
    //(1) add/remove/query BlockEntity
    //(2) spatial relation calculation
    //(We'll move the old function in MapManager to other place because they
    //are not related to Octree. Just a bunch of helper functions)
    //(3) add/remove/query nonUnitBlock(Maybe it's the same as BlockEntity..?)
	

    namespace spatial
    {
        class OctomapOcTree;
        class Octree3DMapManager
        {
        public:

        };
    }
}

#endif // _SPATIAL_NEW_OCTREE3DMAPMANAGER_H

