/*
 * opencog/spatial/Octree.h
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

#ifndef _SPATIAL_OCTREE_H_
#define _SPATIAL_OCTREE_H_

#include <vector>
#include "Octree3DMapManager.h"
#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */

    namespace spatial
    {
        class Block3D;
        class BlockEntity;
        class Octree3DMapManager;

        class Octree
        {
        public:
            Octree(Octree3DMapManager* _om, BlockVector& _nearLeftBottomPoint, Octree * _parent = 0);
            ~Octree();

            // Add a block.This block is not necessary to be a unit block, it can be a bigger block
            // Usually you don't know where to place this block, so the default bool atKnownIndexes is false.
            // Only when atKnownIndexes is true, you assign values for _x, _y and _z.
            // Not that the _x,_y_,z here are not the global position of thi block, it is the child indexes of this tree
            // Strongly recommend not to use atKnownIndexes as true, unless you know this funciton very clearly.
            void addSolidBlock( Block3D * _block,bool atKnownIndexes = false, int _x = 3, int _y = 3, int _z = 3);

            //  Remove an unit block at a given position from the octree system.
            //  return the unit block atom handle.
            Handle removeAnUnitSolidBlock(BlockVector& _pos);

            inline const BlockVector& getNearLeftBottomPoint(){return mNearLeftBottomPoint;}

            inline const AxisAlignedBox& getBoundingBox(){return mBoundingBox;}

            // Check whether this position has been filled by a solid block,
            // _pos is input para, block is output para
            // if solid, return the block in @ block
            bool checkIsSolid(const BlockVector& _pos, Block3D* & _block3d) const;

            // Get the parent octree that contain this _block.
            // Note that this function is only used for finding, so make sure this _block already exists.
            // Only the first parameter is input para, others are all output paras.
            // Return the parent octree, and the indexes of this _block in the mAllMyBlocks[x][y][z] of this parent octree.
            // If this _block is contain in a bigger block, isInsideABigBlock is return truem, and this bigBlock is also returned.
            // If so, the return Octree* and the indexes are all for this bigBlock.
            // If Octree* return 0, it mean this block cannot be found.
            Octree* getParentTreeOfBlock(Block3D* _block, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D* bigBlock);

            // Same to getParentTreeOfBlock. Just the block is a unit block. And you pass the position of it to this funciton.
            // Note that this function is only used for finding, so make sure this block already exists.
            Octree* getParentTreeOfUnitBlock(BlockVector& _pos, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D* bigBlock);

            /** Returns the Depth of this octree, the rootOctree has a Depth 1,
            *   and the subTrees of rootOctree has a Depth 2 and so on...
            */
            inline int getOctreeDepth()
            {
                return mOctreeDepth;
            };

            // Note, the parent of the rootOctree is null
            inline const Octree* getParent()
            {
                return mParent;
            }

            // how many unitblocks per edge of this octree
            inline int getSize()
            {
                return mSize;
            }

            /** Calculate whether this Octree has been full with solid blocks
            */
            bool isFull();

            // Check whehter this octree has become empty.
            // If it has not any subtree or block in it, it is empty.
            bool isEmpty();

            /**  Returns the appropriate indexes for the child octree of this octree into which the box will fit.
            @remarks
            This is used by the OctreeSceneManager to determine which child to traverse next when
            finding the appropriate octree to insert the box.  Since it is a loose octree, only the
            center of the box is checked to determine the octant.
            */
            void getChildIndexes( const AxisAlignedBox & _boundingbox, int &x, int &y, int &z ) const;

            void getChildIndexes( const BlockVector & _pos, int &x, int &y, int &z ) const;

            AxisAlignedBox& getChildBoundingBoxByIndex(int x, int y, int z);

            // When this octree has been full of blocks, then merge all its blocks into a bigger block
            // This will distroy  all the blocks inside, and all the atoms of the blocks will be moved to this new big block.
            // return this bigger block.
            Block3D*  mergeAllMyBlocks();

            // break this a block into 8 smaller blocks
            // Note that when this block is an unit block, you cannot do this
            // This function is usually used when a unit block is removed from a composed block
            // x,y,z is the indexes of this block to be break in mAllMyBlocks[x][y][z];
            void breakBlockInto8Blocks(int x, int y, int z);

            // return its indexes in its parent if has a parent
            inline void getIndexesInParent(int& x, int& y, int& z)
            {
                if (mParent != 0)
                {
                    x = mIndex_x;
                    y = mIndex_y;
                    z = mIndex_z;
                }
            }

            // find all the blocks combine with the block in _pos,
            // the return list does not contain the block in this _pos
            vector<Block3D*>  findAllBlocksCombinedWith(BlockVector* _pos, bool useBlockMaterial = true);

            // get all the existing BlockEntities will combined by this block (if add a block in this _pos)
            // calculate all the 26 neighbours
            vector<BlockEntity*> getNeighbourEntities(BlockVector& _pos);

            // get any a Neighbour Solid BlockVector of curPos
            // the neighbourBlock will return the block contains this neighbour BlockVector
            BlockVector getNeighbourSolidBlockVector(BlockVector& curPos , Block3D* &neighbourBlock);

            // return all the neighbour sold BlockVector of curPos
            vector<BlockVector> getAllNeighbourSolidBlockVectors(BlockVector& curPos);

            // one of the 8 parts of a octree, if is not null,
            // it is either a sub octree in mChildren or a block3d in mAllMyBlocks
            /** 3D array of children of this octree.
            @remarks
            Children are dynamically created as needed when blocks are inserted in the Octree.
            the index of the children is ordered by [x][y][z]
            */
            Octree * mChildren[2][2][2];

            // a block here only means one of the 8 blocks in this octree Depth, not a unit block.
            Block3D * mAllMyBlocks[2][2][2];

            // the Octree3DMapManager
            Octree3DMapManager* mOctree3DMapManager;

            Octree* clone(Octree3DMapManager* newOctree3DMapManager, Octree* parentTree = 0);

        protected:

            //parent octree
            Octree * mParent;

            // the size of this Octree, means how many units per edge of this cube
            int mSize;

            // octree Depth:the rootOctree has a Depth 1, and the subTrees of rootOctree has a Depth 2 and so on...
            int mOctreeDepth;

            /** The bounding box of the octree
            */
            AxisAlignedBox mBoundingBox;

            // the nearLeftBottom of this tree
            BlockVector mNearLeftBottomPoint;

            // the central point of this tree
            BlockVector mCentre;

            // the indexes in its parent if has a parent
            int mIndex_x, mIndex_y, mIndex_z;

            // this constructor is only used in clone this instance
            Octree(Octree3DMapManager* _octree3DMapManager, Octree* _mParent, int _size, int _OctreeDepth, AxisAlignedBox& _boundingBox,
                           BlockVector& _nearLeftBottomPoint,BlockVector& _centre, int index_x, int index_y, int index_z ):
                mOctree3DMapManager(_octree3DMapManager),mParent(_mParent), mSize(_size),mOctreeDepth(_OctreeDepth),mBoundingBox(_boundingBox),
                mNearLeftBottomPoint(_nearLeftBottomPoint),mCentre(_centre),mIndex_x(index_x), mIndex_y(index_y), mIndex_z(index_z){}

        };
    }
/** @}*/
}

#endif // _SPATIAL_OCTREE_H_
