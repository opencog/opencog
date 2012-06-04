/*
 * opencog/spatial/BlockEntity.cc
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

#include "BlockEntity.h"
#include "Octree3DMapManager.h"
#include <opencog/util/StringManipulator.h>


using namespace opencog;
using namespace opencog::spatial;

int BlockEntity::BlockEntityIDCount = 0;

BlockEntity::BlockEntity(Block3D& firstBlock, std::string entityName)
{
    mMyBlocks.push_back(&firstBlock);
    firstBlock.mBlockEntity = this;
    mBoundingBox = firstBlock.getBoundingBox();
    mID = ++ BlockEntityIDCount;
    mName = entityName;
    mYaw = 0.0f; // currently, we don't consider about the rotation of block entities

    // blocks are always obstacle
    is_obstacle = true;

    _ReCalculatCenterPosition();

    Octree3DMapManager::newAppearBlockEntityList.push_back(this);
    Octree3DMapManager::updateBlockEntityList.push_back(this);

}

BlockEntity::BlockEntity(vector<Block3D*>& blockList, std::string entityName)
{
    mBoundingBox = ((Block3D*)(blockList.front()))->getBoundingBox();
    addBlocks(blockList);
    mID = ++ BlockEntityIDCount;
    mName = entityName;

    Octree3DMapManager::newAppearBlockEntityList.push_back(this);
}

BlockEntity::~BlockEntity()
{
    Octree3DMapManager::newDisappearBlockEntityList.push_back(mEntityNode);
}

void BlockEntity::addBlock(Block3D* _block)
{
    mMyBlocks.push_back(_block);
    mBoundingBox += _block->getBoundingBox();
    _block->mBlockEntity = this;
    _ReCalculatCenterPosition();
}

void BlockEntity::addBlocks(vector<Block3D*>& _blockList)
{
    vector<Block3D*>::iterator iter = _blockList.begin();
    for(; iter != _blockList.end(); ++ iter)
    {
        addBlock((Block3D*)(*iter));
    }
    _ReCalculatCenterPosition();
}

void BlockEntity::removeBlock(Block3D* _block)
{
    vector<Block3D*>::iterator iter;
    for (; iter != mMyBlocks.end(); ++ iter)
    {
        if (_block == *iter)
        {
            _block->mBlockEntity = 0;
            mMyBlocks.erase(iter);
            break;
        }
    }

    _reCalculateBoundingBox();
}

void BlockEntity::_ReCalculatCenterPosition()
{
    mCenterPosition.x = mBoundingBox.nearLeftBottomConer.x + mBoundingBox.size_x / 2;
    mCenterPosition.y = mBoundingBox.nearLeftBottomConer.y + mBoundingBox.size_y / 2;
    mCenterPosition.z = mBoundingBox.nearLeftBottomConer.z + mBoundingBox.size_z / 2;
}

void BlockEntity::_reCalculateBoundingBox()
{
    mBoundingBox = ((Block3D*)mMyBlocks.front())->getBoundingBox();
    vector<Block3D*>::iterator iter;
    iter ++;
    for (; iter != mMyBlocks.end(); ++ iter)
    {
        mBoundingBox += ((Block3D*)(*iter))->getBoundingBox();
    }

    _ReCalculatCenterPosition();
}

// sort the block order, make the blocks in the this entity from the bottom to top
void BlockEntity::SortBlockOrder()
{
    int size = mMyBlocks.size();
    if (size < 2) // only one block in this entity, not need to sort
        return;


    Block3D* blocks[size];

    vector<Block3D*>::iterator iter = mMyBlocks.begin();
    int i = 0;
    for (; iter != mMyBlocks.end(); ++ iter)
    {
        blocks[i] = (Block3D*)(*iter);
        i ++;
    }

    Block3D* tmp;
    for (int k = 0; k < size; k++)
    for (int j = 0; j < size - 1 -k; j ++)
    {

        if (blocks[j] > blocks[j+1])
        {
            tmp = blocks[j];
            blocks[j] = blocks[j+1];
            blocks[j+1] = tmp;
        }
    }

    mMyBlocks.clear();

    for (int x = 0; x < size; x++)
        mMyBlocks.push_back(blocks[x]);

}

void BlockEntity::clearAllBlocks()
{
    // first we should tag all the blocks in this entity as "not belong to any entity"
    vector<Block3D*>::iterator it;
    for (it = mMyBlocks.begin(); it != mMyBlocks.end(); ++it)
    {
        Block3D* b = *it;
        b->mBlockEntity = 0;
    }

    mMyBlocks.clear();
    mBoundingBox = AxisAlignedBox::ZERO;

}

