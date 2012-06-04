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

#ifndef _SPATIAL_BLOCKENTITY_H
#define _SPATIAL_BLOCKENTITY_H

#include "Block3DMapUtil.h"
#include "Block3D.h"
#include "Entity3D.h"
#include <vector>

namespace opencog
{
    namespace spatial
    {
        class Block3D;
        class Entity3D;

        // A blockEntity is made of blocks
        class BlockEntity : public Entity3D
        {
        protected:
            static int BlockEntityIDCount;
        public:
            // this will also add a node to the atom space representing this blockEntity
            BlockEntity(Block3D& firstBlock, std::string entityName = "");
            BlockEntity(vector<Block3D*>& blockList, std::string entityName = "");
            ~BlockEntity();

            inline const vector<Block3D*>& getBlockList(){return mMyBlocks;}

            void addBlock(Block3D* _block);
            void addBlocks(vector<Block3D*>& _blockList);
            void removeBlock(Block3D* _block);
            void SortBlockOrder();

            // when this entity has some changes, we need to clear all the blocks in this entity and refind all its blocks
            void clearAllBlocks();

            bool isBlockEntity(){return true;}

        protected:

            vector<Block3D*> mMyBlocks; // all the blocks in this entity
            void _reCalculateBoundingBox();
            void _ReCalculatCenterPosition();

        };
    }
}


#endif // _SPATIAL_BLOCKENTITY_H
