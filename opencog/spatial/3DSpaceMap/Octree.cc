
/*
 * opencog/spatial/3DSpaceMap/Octree.cc
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

#include "Octree.h"
#include "Block3D.h"
#include "BlockEntity.h"
#include "Octree3DMapManager.h"
#include <iterator>
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::spatial;

Octree::Octree(Octree3DMapManager* _om, BlockVector& _nearLeftBottomPoint, Octree * _parent):
    mOctree3DMapManager(_om) ,mParent(_parent), mNearLeftBottomPoint(_nearLeftBottomPoint)
{
    if (_parent == NULL)
    {
        // this tree is the root tree
        mOctreeDepth = 1;

        // calculate the size of this tree, how many unit blocks in each edge
        int power = mOctree3DMapManager->getTotalDepthOfOctree();
        mSize = 1;
        for (int i = 0; i < power; ++i)
            mSize *= 2;
    }
    else
    {
        // this tree is not the root tree
        mOctreeDepth = mParent->getOctreeDepth() + 1;
        mSize = mParent->getSize()/2;
    }

    mBoundingBox.nearLeftBottomConer = _nearLeftBottomPoint;
    mBoundingBox.size = mSize;
    mBoundingBox.size_x = mSize;
    mBoundingBox.size_y = mSize;
    mBoundingBox.size_z = mSize;

    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                mChildren[x][y][z] = 0;
                mAllMyBlocks[x][y][z] = 0;
            }

    if (mParent!= 0)
    {
        mParent->getChildIndexes(mBoundingBox, mIndex_x,mIndex_y,mIndex_z);
    }

    mCentre.x = mNearLeftBottomPoint.x + mSize/2;
    mCentre.y = mNearLeftBottomPoint.y + mSize/2;
    mCentre.z = mNearLeftBottomPoint.z + mSize/2;

}

//Octree::Octree(){}

Octree::~Octree()
{
    // delete itself at the same time delete all its children octrees recursively
    if (mParent != 0)
    {
        mParent->mChildren[mIndex_x][mIndex_y][mIndex_z] = 0;
    }

    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                if (mChildren[x][y][z] != 0)
                    delete mChildren[x][y][z];

                if (mAllMyBlocks[x][y][z] != 0)
                    delete mAllMyBlocks[x][y][z];
            }
}

// Note that this block is not necessary to be a unit block, it can be a bigger block
void Octree::addSolidBlock(Block3D * _block, bool byKnownIndexes, int _x, int _y, int _z)
{
    int x,y,z;
    Octree* tree = this;

    if (byKnownIndexes)
    {
        // if already know which position to put this block
        x = _x;
        y = _y;
        z = _z;
    }
    else
    {
        // if don't know where to put this block
        // first, find the exactly sub-tree that this block will belong to.

        // if this block will fill this tree itself, then we take the parent of this tree
        if (_block->getBoundingBox() == tree->getBoundingBox())
        {
            if (this->mOctreeDepth != 1)
                tree = this->mParent;
            else
            {
                logger().error("This block is too big, and it will occupy the whole space. Could not add such a block!/n");
                return;
            }

            tree->getChildIndexes( this->mBoundingBox, x , y, z);
        }
        else
        {
            int count = 0;
            while(true)
            {
                if (count++ > mOctree3DMapManager->getTotalDepthOfOctree())
                {
                    // usually it should not go here. This to prevent this while loop from not stopping
                    logger().error("Cannot find a proper position for this block at x = %d, y = %d, z= %d ! /n",
                                   _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
                    return;
                }

                tree->getChildIndexes(_block->getBoundingBox(), x,y,z);

                AxisAlignedBox childBox = tree->getChildBoundingBoxByIndex(x,y,z);

                if (_block->getBoundingBox() == childBox )
                {
                    if (tree->mChildren[x][y][z] != 0)
                    {
                        // Usually, it should not come here
                        // If it comes here, it means this child-tree is not empty,
                        // so you cannot add a new block that as big as this child tree to it.
                        logger().error("There are already block in this postion! Cannot add a block at x = %d, y = %d, z= %d ! /n",
                                      _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
                        return;
                    }

                    // this childtree is the exactly tree that this block will fill it
                    // break with this parent tree and x,y,z indexes for its child tree
                    break;
                }
                else
                {
                    // if this child tree does not exist, creat it
                    if (tree->mChildren[x][y][z] == 0)
                        tree->mChildren[x][y][z] = new Octree(mOctree3DMapManager, childBox.nearLeftBottomConer ,tree);

                    // continue to find a smaller child tree
                    tree = tree->mChildren[x][y][z];
                }

            }
        }

    }

    // until here, we've got the parent tree of this block and its child indexes of this block.
    // Now we can add it into the proper place
    if (tree->mChildren[x][y][z] == 0)
    {
        tree->mAllMyBlocks[x][y][z] = _block;

        // After adding this block, we'll try to merge this tree into a bigger block if it is full.
        Block3D* bigBlock = tree->mergeAllMyBlocks();

        if (bigBlock != 0)
        {
            // This tree happens to be full after adding this _block, and has been merged into a bigger block.
            // Destroy this tree, and add this big block to the proper place
            Octree* parentTree = (Octree*)(tree->getParent());
            tree->getIndexesInParent(x,y,z);
            delete tree;// Note: tree has been delete here, and this tree is possible to be this, so don't reference to this next

            parentTree->addSolidBlock(bigBlock,true,x,y,z);
        }

    }
    else
    {
        // Usually, it should not come here
        // If it comes here, it means this child-tree is not empty,
        // so you cannot add a new block that as big as this child tree to it.
        logger().error("There are already block in this postion! Cannot add a block at x = %d, y = %d, z= %d ! /n",
                      _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
        return;
    }

}

Handle Octree::removeAnUnitSolidBlock(BlockVector& _pos)
{
    Handle h = mOctree3DMapManager->getUnitBlockHandleFromPosition(_pos);
    if (h == Handle::UNDEFINED)
        return h;

    // first, check if this _pos is inside this octree
    if (! mBoundingBox.isUnitBlockInsideMe(_pos))
    {
        // Usually, it should not come here
        logger().error("You want to remove a unit block from otree, but the block in not inside this octree: block is at x = %d, y = %d, z= %d ! /n",
                      _pos.x,_pos.y,_pos.z);
        return Handle::UNDEFINED;
    }

    // second, find eigher a bigger block that contains this block,
    // or a sub-octree that contain this block
    int x,y,z;
    bool isInsideABigblock;
    Block3D* bigBlock = NULL;
    Octree* tree = getParentTreeOfUnitBlock(_pos, x,y,z, isInsideABigblock, bigBlock);

    if (tree == 0)
    {
        // Usually, it should not come here
        logger().error("You want to remove a unit block from otree,but cannot find this block x = %d, y = %d, z= %d ! /n",
                      _pos.x,_pos.y,_pos.z);
        return Handle::UNDEFINED;
    }

    if (! isInsideABigblock)
    {
        // This unit block is not contained in a bigger block, we can remove it more easily.
        delete (tree->mAllMyBlocks[x][y][z]);
        tree->mAllMyBlocks[x][y][z] = 0;

        // check if this sub-octree is empty after this block removed, and check this for every of its parents.
        while(true)
        {
            if (tree->isEmpty())
            {
                if (tree->getOctreeDepth() != 1) // this tree is not the root
                {
                    tree->getIndexesInParent(x,y,z);
                    tree = (Octree*) (tree->getParent());

                    delete (tree->mChildren[x][y][z]);
                    tree->mChildren[x][y][z] = 0;
                    return h;
                }
                else
                    break;
            }
            else
                break;

        }
    }
    else
    {
        // This unit block is contained in a bigger block, we should break the bigger block into smaller blocks first
        // And we must break the blocks recursively to the unit level
        int level = bigBlock->getLevel();
        AxisAlignedBox atomBox(_pos);

        while (true)
        {
            tree->breakBlockInto8Blocks(x,y,z);


            tree = tree->mChildren[x][y][z];
            tree->getChildIndexes(atomBox, x,y,z);

            if ( --level == 1)
                break;
        }

        delete tree->mAllMyBlocks[x][y][z];
        tree->mAllMyBlocks[x][y][z] = 0;
        return h;
    }

    return Handle::UNDEFINED;
}

AxisAlignedBox& Octree::getChildBoundingBoxByIndex(int x, int y, int z)
{
    static AxisAlignedBox childBox;
    childBox.nearLeftBottomConer.x = mNearLeftBottomPoint.x + x * mSize/2;
    childBox.nearLeftBottomConer.y = mNearLeftBottomPoint.y + y * mSize/2;
    childBox.nearLeftBottomConer.z = mNearLeftBottomPoint.z + z * mSize/2;

    childBox.size = mSize/2;
    childBox.size_x = childBox.size;
    childBox.size_y = childBox.size;
    childBox.size_z = childBox.size;
    return childBox;
}

bool Octree::checkIsSolid(const BlockVector& _pos, Block3D* & _block3d) const
{
    Octree* tree = (Octree*)(mOctree3DMapManager->getRootOctree());

    int x,y,z;
    while(true)
    {
        tree->getChildIndexes(_pos, x,y,z);

        if (tree->mAllMyBlocks[x][y][z] != 0)
        {
            _block3d = tree->mAllMyBlocks[x][y][z];
            return true;
        }
        else if (tree->mChildren[x][y][z] == 0)
            return false;
        else
            tree = tree->mChildren[x][y][z];
    }

}

Octree* Octree::getParentTreeOfBlock(Block3D* _block, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D* bigBlock)
{
    Octree* tree = this;

    int count = 0;
    while(true)
    {
        if (count++ > mOctree3DMapManager->getTotalDepthOfOctree())
        {
            // usually it should not go here. This to prevent this while loop from not stopping
            logger().error("Cannot find this block at x = %d, y = %d, z= %d ! /n",
                           _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
            return 0;
        }

        tree->getChildIndexes(_block->getBoundingBox(), x,y,z);

        AxisAlignedBox childBox = tree->getChildBoundingBoxByIndex(x,y,z);

        if (_block->getBoundingBox() == childBox )
        {
            if ( tree->mAllMyBlocks[x][y][z] != 0)
            {
                // find!
                isInsideABigBlock = false;
                return tree;
            }
            else
            {
                // Usually, it should not come here
                // If it comes here, it means this block does not exist
                logger().error("A block you try to find at x = %d, y = %d, z= %d, does not exist! /n",
                              _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
                return 0;
            }

        }
        else
        {

            if (tree->mChildren[x][y][z] != 0)
            {
                // continue to find in a smaller child tree
                tree = tree->mChildren[x][y][z];
            }
            else if (tree->mAllMyBlocks[x][y][z] != 0)
            {
                // this block is contained in a bigger block and we find this biggeer block
                isInsideABigBlock = true;
                bigBlock = tree->mAllMyBlocks[x][y][z];
                return tree;
            }
            else
            {
                // Usually, it should not come here
                // If it comes here, it means this block does not exist
                logger().error("A block you try to find at x = %d, y = %d, z= %d, is supposed to be in a sub-octree,but this tree does not exist! /n",
                              _block->getPosition().x,_block->getPosition().y,_block->getPosition().z);
                return 0;
            }
        }

    }

}

Octree* Octree::getParentTreeOfUnitBlock(BlockVector& _pos, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D* bigBlock)
{
    Block3D unitBlock(1, _pos);
    Octree* tree = getParentTreeOfBlock(&unitBlock, x,y,z,isInsideABigBlock,bigBlock);
    return tree;
}

void Octree::getChildIndexes( const AxisAlignedBox & _boundingbox, int &x, int &y, int &z ) const
{
    if (_boundingbox.nearLeftBottomConer.x >= mCentre.x)
        x = 1;
    else
        x = 0;

    if (_boundingbox.nearLeftBottomConer.y >= mCentre.y)
        y = 1;
    else
        y = 0;

    if (_boundingbox.nearLeftBottomConer.z >= mCentre.z)
        z = 1;
    else
        z = 0;
}

void Octree::getChildIndexes( const BlockVector & _pos, int &x, int &y, int &z ) const
{
    if (_pos.x >= mCentre.x)
        x = 1;
    else
        x = 0;

    if (_pos.y >= mCentre.y)
        y = 1;
    else
        y = 0;

    if (_pos.z >= mCentre.z)
        z = 1;
    else
        z = 0;
}

bool Octree::isFull()
{
    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                if (mAllMyBlocks[x][y][z] == 0)
                    return false;
            }
    return true;
}

bool Octree::isEmpty()
{
    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                if (mAllMyBlocks[x][y][z] != 0 || mChildren[x][y][z] != 0)
                    return false;
            }
    return true;
}

Block3D* Octree::mergeAllMyBlocks()
{
    // Make sure this tree is not the root tree
    if (mOctreeDepth == 1)
        return 0;

    // First, check whether this octee has been full with blocks
    // if it is not full, should not merge.
    if (! isFull())
        return 0;

    // We must also check whether all the blocks are in the same BlockMaterial
    // if they are not in the same BlockMaterial, cannot perform merge.
    BlockMaterial myMaterial = (mAllMyBlocks[0][0][0])->getBlockMaterial();

    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                if ((mAllMyBlocks[x][y][z])->getBlockMaterial() != myMaterial)
                    return 0;
            }

    // Now begin to merge
    // Create a new big block, as big as this octree
    int blockLevel = mOctree3DMapManager->getTotalDepthOfOctree() - mOctreeDepth + 2;

    if (blockLevel <= 1)
    {
        // Never arrive here, unless there is an error.
        logger().error("Merge blocks error: the new big block will be an unit block at x = %d, y = %d, z= %d ! /n",
                      mNearLeftBottomPoint.x,mNearLeftBottomPoint.y,mNearLeftBottomPoint.z);
        return 0;
    }

    Block3D* newBlock = new Block3D(blockLevel, mNearLeftBottomPoint, myMaterial.materialType, myMaterial.color);

    // Get the blockEntity these blocks belong to
    BlockEntity* myEntity = mAllMyBlocks[0][0][0]->mBlockEntity;

    // delete all the old blocks in this octree.
    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                delete (mAllMyBlocks[x][y][z]);
                mAllMyBlocks[x][y][z] = 0;
            }

    // add this new big block to the BlockEntity
    if (myEntity != 0)
        myEntity->addBlock(newBlock);

    return newBlock;
}

void Octree::breakBlockInto8Blocks(int x, int y, int z)
{
    Block3D* bigBlock = mAllMyBlocks[x][y][z];
    mAllMyBlocks[x][y][z] = 0;

    // create a new subtree for the new 8 blocks
    mChildren[x][y][z] = new Octree(mOctree3DMapManager, (BlockVector&)(bigBlock->getPosition()), this);

    // Get the BlockEntity this block belongs to
    BlockEntity* myEntity = bigBlock->mBlockEntity;

    // create 8 new blocks
    for (int i = 0; i < 2; i ++)
        for (int j = 0; j < 2; j ++)
            for (int k = 0; k < 2; k ++)
            {

                ((mChildren[x][y][z])->mAllMyBlocks)[i][j][k] = new Block3D(bigBlock->getLevel()-1,
                                                              (mChildren[x][y][z])->getChildBoundingBoxByIndex(i,j,k).nearLeftBottomConer,
                                                              bigBlock->getBlockMaterial().materialType,
                                                              bigBlock->getBlockMaterial().color);
                if (myEntity != 0)
                    myEntity->addBlock(((mChildren[x][y][z])->mAllMyBlocks)[i][j][k]);
            }

    // Move all the unit blocks in this big block to the right small blocks
//    vector<BlockVector> unitBlocks = bigBlock->getAllMyUnitBlockVectors();
//    vector<BlockVector>::iterator iter = unitBlocks.begin();
//    BlockVector atomPos;
//    int o,p,q;
//    map<Handle, BlockVector> AllUnitBlockatoms = (map<Handle, BlockVector>&)(mOctree3DMapManager->getAllUnitBlockatoms());
//    Handle handle;
//    for (; iter != unitBlockAtoms.end(); iter ++)
//    {
//        handle = (Handle)(*iter);
//        atomPos = (BlockVector)(AllUnitBlockatoms[handle]);
//        AxisAlignedBox atomBox(atomPos);
//        (mChildren[x][y][z])->getChildIndexes(atomBox, o,p,q);
//        (mChildren[x][y][z])->mAllMyBlocks[o][p][q]->addBlockAtom(*iter);
//    }

    delete bigBlock;
}

// we don't contain the begin block in our return list
vector<BlockEntity*> Octree::getNeighbourEntities(BlockVector& _pos)
{
    vector<BlockEntity*> entities;
    vector<BlockEntity*>::iterator it;
    AxisAlignedBox rootBox = (AxisAlignedBox&)(mOctree3DMapManager->getMapBoundingBox());
    Block3D* beginBlock;
    checkIsSolid(_pos, beginBlock);

    // check the 26 neighbours unit blockvectors of every block combined
    Block3D* block;

    for (int i = -1; i < 2; i ++)
        for (int j = -1; j < 2; j ++)
            for (int k = -1; k < 2; k ++)
            {
                if (i == 0 && j == 0 && k == 0)
                    continue;
                BlockVector nextPos(_pos.x + i,_pos.y + j, _pos.z + k);

                if (! rootBox.isUnitBlockInsideMe(nextPos))
                    continue;

                if (checkIsSolid(nextPos, block))
                {
                    // there is a block in this neighbour pos
                    if (block == beginBlock)
                        continue; // we don't contain the begin block in our return list

                    for (it = entities.begin(); it != entities.end(); ++ i )
                    {
                        if (block->mBlockEntity == (BlockEntity*)(*it))
                            break;
                    }

                    // this block has not been push into result entity list,
                    // so push it back
                    if (it == entities.end())
                        entities.push_back(block->mBlockEntity);
                }
            }

    return entities;
}


vector<Block3D*> Octree::findAllBlocksCombinedWith(BlockVector* _pos, bool useBlockMaterial)
{
    vector<Block3D*> blockList;
    vector<Block3D*>::iterator iter;
    vector<BlockVector> searchList, searchedList;
    vector<BlockVector>::iterator it, it2;
    searchList.push_back(*_pos);

    AxisAlignedBox rootBox = (AxisAlignedBox&)(mOctree3DMapManager->getMapBoundingBox());

    // check the 26 neighbours unit blockvectors of every block combined
    // by a breadth-first searching
    BlockVector curPos;

    Block3D* curblock;

    Block3D* nextBlock;

    BlockMaterial curMaterial;

    while(searchList.size() != 0)

    {
        curPos = searchList.front();
        if (checkIsSolid(curPos, curblock))
        {

            curMaterial = curblock->getBlockMaterial();

            for (int i = -1; i < 2; i ++)
                for (int j = -1; j < 2; j ++)
                    for (int k = -1; k < 2; k ++)
                    {
                        if (i == 0 && j == 0 && k == 0)
                            continue;
                        BlockVector nextPos(curPos.x + i, curPos.y + j, curPos.z + k);

                        if (! rootBox.isUnitBlockInsideMe(nextPos))
                        {
                            searchedList.push_back(nextPos);
                            continue;
                        }

                        // if this nextPos already searched, don't push in searchList
                        for(it = searchedList.begin(); it != searchedList.end(); it ++)
                        {
                            if (nextPos == (BlockVector)(*it))
                                break;
                        }
                        if (it != searchedList.end())
                            continue;

                        if (checkIsSolid(nextPos, nextBlock) && ( (!useBlockMaterial) || (nextBlock->getBlockMaterial() == curMaterial)))
                        {
                            for(it2 = searchList.begin(); it2 != searchList.end(); it2 ++)
                            {
                                if (nextPos == (BlockVector)(*it2))
                                    break;
                            }
                            if (it2 == searchList.end())
                                searchList.push_back(nextPos);

                            if (nextBlock->mBlockEntity == 0)
                            {
                                for(iter = blockList.begin(); iter != blockList.end(); iter ++)
                                {
                                    if (nextBlock == (Block3D*)(*iter))
                                        break;
                                }
                                if (iter == blockList.end())
                                    blockList.push_back(nextBlock);
                            }
                        }
                        else
                        {
                            searchedList.push_back(nextPos);
                        }
                    }
        }
        searchedList.push_back(curPos);
        searchList.erase(searchList.begin());

    }

    return blockList;

}

BlockVector Octree::getNeighbourSolidBlockVector(BlockVector& curPos, Block3D* &neighbourBlock)
{
    // check 26 neighbours
    AxisAlignedBox rootBox = (AxisAlignedBox&)(mOctree3DMapManager->getMapBoundingBox());

        for (int i = -1; i < 2; i ++)
            for (int j = -1; j < 2; j ++)
                for (int k = -1; k < 2; k ++)
                {
                    if (i == 0 && j == 0 && k == 0)
                        continue;
                    BlockVector nextPos(curPos.x + i, curPos.y + j, curPos.z + k);

                    if (! rootBox.isUnitBlockInsideMe(nextPos))
                        continue;

                    if ( checkIsSolid(nextPos,neighbourBlock))
                    {
                        return nextPos;
                    }
                }

        return BlockVector::ZERO;


}

vector<BlockVector> Octree::getAllNeighbourSolidBlockVectors(BlockVector& curPos)
{
    vector<BlockVector> vectorList;

    Block3D* neighbourBlock;

    // check 26 neighbours
    AxisAlignedBox rootBox = (AxisAlignedBox&)(mOctree3DMapManager->getMapBoundingBox());

        for (int i = -1; i < 2; i ++)
            for (int j = -1; j < 2; j ++)
                for (int k = -1; k < 2; k ++)
                {
                    if (i == 0 && j == 0 && k == 0)
                        continue;
                    BlockVector nextPos(curPos.x + i, curPos.y + j, curPos.z + k);

                    if (! rootBox.isUnitBlockInsideMe(nextPos))
                        continue;

                    if ( checkIsSolid(nextPos,neighbourBlock))
                    {
                        vectorList.push_back(nextPos);
                    }
                }

        return vectorList;
}

Octree* Octree::clone(Octree3DMapManager* newOctree3DMapManager, Octree *parentTree)
{
    Octree* cloneOctree = new Octree(newOctree3DMapManager,parentTree,mSize,mOctreeDepth,mBoundingBox,mNearLeftBottomPoint,mCentre,mIndex_x,mIndex_y,mIndex_z);

    for (int x = 0; x < 2; x ++)
        for (int y = 0; y < 2; y ++)
            for (int z = 0; z < 2; z ++)
            {
                // clone subtrees
                if (mChildren[x][y][z] == 0)
                    cloneOctree->mChildren[x][y][z] = 0;
                else
                    cloneOctree->mChildren[x][y][z] = (mChildren[x][y][z])->clone(newOctree3DMapManager,cloneOctree);

                // clone the blocks inside me
                if (mAllMyBlocks[x][y][z] == 0)
                    cloneOctree->mAllMyBlocks[x][y][z] = 0;
                else
                    cloneOctree->mAllMyBlocks[x][y][z] = mAllMyBlocks[x][y][z]->clone();
            }

    return cloneOctree;

}
