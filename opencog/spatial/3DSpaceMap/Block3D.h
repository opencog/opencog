/*
 * opencog/spatial/3dBlock.h
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

#ifndef _SPATIAL_BLOCK3D_H_
#define _SPATIAL_BLOCK3D_H_

#include "Block3DMapUtil.h"
#include <vector>

using namespace std;

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        /**
         * Note that this Block3D is not necessarily a minimum unit block.
         * It can be a bigger cube block composed by 8 or 8 to the nth power unit blocks.
         */
        class BlockEntity;

        class Block3D
        {

        public:
            /**
             * @ _composedLevel: It indicates how big this block is.
             *                   Level for a unit block is 1, level for a block composed by 8 units is 2, 64 unitts is 3
             * @ _postion: The position of this block in block3DSpaceMap (the near-left-bottom point of this block).
             * @ _unitBlockAtom: Only when this block is an unit block , pass a valid atom that represent this block in the Atomspace
             */
            Block3D(int _composedLevel, BlockVector& _position, string _materialType = "", string _color = "",bool _canDestroy = true);

            ~Block3D();

            Block3D* clone();

            inline int getLevel(){return mLevel;}

            inline const BlockVector& getPosition(){return mPosition;}

            inline const BlockMaterial& getBlockMaterial(){return mBlockMaterial;}

            inline const AxisAlignedBox& getBoundingBox(){return mBoundingBox;}

            inline bool canDestroy(){return mCanDestroy;}

            //vector<BlockVector> getAllMyUnitBlockVectors();

            // The BlockEntity this Block belongs to, it is defaultly null
            BlockEntity* mBlockEntity;

        protected:

            // It indicates how big this block is.
            // Level for a unit block is 1, level for a block composed by 8 units is 2...
            int mLevel;

            // The global position of this block (the left-bottom point of this block)
            BlockVector mPosition;

            // Material of this block
            BlockMaterial mBlockMaterial;

            /** The bounding box of this block
            */
            AxisAlignedBox mBoundingBox;

            // whether this block can be destroyed
            bool mCanDestroy;

        };
    }
/** @}*/
}

#endif // _SPATIAL_BLOCK3D_H_
