/*
 * opencog/spatial/BlockEntity.h
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

#ifndef _SPATIAL_BlockEntity_H
#define _SPATIAL_BlockEntity_H

#include "Block3DMapUtil.h"
#include "Block3D.h"
#include "Entity3D.h"
#include "Octree3DMapManager.h"
#include <vector>
#include <map>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        class Block3D;
        class Entity3D;

        // the info to describe the adjacent situation with an neighbour blockEntity
        struct AdjacentInfo
        {
            int adjacentUnitNum; // the number of unit blocks in me which adjacent to this neighbour blockEntity
        };

        // A BlockEntity is made of blocks
        class BlockEntity : public Entity3D
        {
        protected:
            static int BlockEntityIDCount;
        public:
            // this will also add a node to the atom space representing this BlockEntity
            BlockEntity(Octree3DMapManager* map,Block3D& firstBlock, std::string entityName = "");
            BlockEntity(Octree3DMapManager* map,vector<Block3D*>& blockList, std::string entityName = "");
            BlockEntity(Octree3DMapManager* map,vector<BlockEntity*> subEntities, std::string entityName = ""); // for super BlockEntity only
            ~BlockEntity();

            // Make sure a new octree has been cloned and all the blocks have been cloned before calling this function
            BlockEntity* clone(Octree3DMapManager* _newSpaceMap);

            inline const vector<Block3D*>& getBlockList(){return mMyBlocks;}
            inline const vector<BlockEntity*>& getSubEntities() {return mMySubEntities;}
            inline const BlockEntity* getFartherEntity() {return mFartherEntity;}
            inline bool isSuperBlockEntity(){return is_superBlockEntity;}

            void addBlock(Block3D* _block);
            void addBlocks(vector<Block3D*>& _blockList);
            void removeBlock(Block3D* _block);
            void SortBlockOrder();

            // when this entity has some changes, we need to clear all the blocks in this entity and refind all its blocks
            void clearAllBlocks();

            bool isBlockEntity(){return true;}

            void setFartherEntity(BlockEntity* father){mFartherEntity = father;}

            void addSubEntity(BlockEntity* subEntity);
            void addSubEntities(vector<BlockEntity*> subEntityList);

            void removeSubEntity(BlockEntity* subEntityToRemove);
            void removeSubEntities(vector<BlockEntity*> subEntityListToRemove);

            // only for non-super blockEntities
            map<BlockEntity*,AdjacentInfo> NeighbourBlockEntities;

        protected:
            Octree3DMapManager* spaceMap;
            bool is_superBlockEntity; // superBlockEntity is a BlockEntity composed of sub blockEntities
            BlockEntity* mFartherEntity; // the farther BlockEntity of this entity
            vector<Block3D*> mMyBlocks; // all the blocks in this entity
            vector<BlockEntity*> mMySubEntities; // all the subBlockEntities in this entity

            void _init(Octree3DMapManager* map,std::string entityName, bool _is_superBlockEntity);
            void _reCalculateBoundingBox();
            void _ReCalculatCenterPosition();

            // this constructor is only for clone this instance (clone it from a spaceMap to another spaceMap)
            BlockEntity(Octree3DMapManager* _newSpaceMap, bool _is_superBlockEntity,BlockEntity* _FartherEntity,
                        int _ID, std::string _Name, AxisAlignedBox& _BoundingBox, BlockVector& _CenterPosition,vector<Block3D*>& _MyBlocks,vector<BlockEntity*>& _MySubEntities);

        };
    }
/** @}*/
}


#endif // _SPATIAL_BlockEntity_H
