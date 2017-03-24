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
#include <opencog/util/Logger.h>

using namespace opencog;
using namespace opencog::spatial;

int BlockEntity::BlockEntityIDCount = 0;

void BlockEntity::_init(Octree3DMapManager* map,std::string entityName, bool _is_superBlockEntity)
{
    this->spaceMap = map;
    mID = ++ BlockEntityIDCount;
    mName = entityName;
    mEntityClass = "block";
    mYaw = 0.0f; // currently, we don't consider about the rotation of block entities

    // blocks are always obstacle
    is_obstacle = true;
    mFartherEntity = 0;

    is_superBlockEntity = _is_superBlockEntity;

    spaceMap->newAppearBlockEntityList.push_back(this);

    if (_is_superBlockEntity)
        spaceMap->updateSuperBlockEntityList.push_back(this);
    else
        spaceMap->updateBlockEntityList.push_back(this);
}

BlockEntity::BlockEntity(Octree3DMapManager* map,Block3D& firstBlock, std::string entityName)
{
    mMyBlocks.push_back(&firstBlock);
    firstBlock.mBlockEntity = this;
    mBoundingBox = firstBlock.getBoundingBox();

    _ReCalculatCenterPosition();

    _init(map,entityName, false);

}

BlockEntity::BlockEntity(Octree3DMapManager* map,vector<Block3D*>& blockList, std::string entityName)
{
    mBoundingBox = ((Block3D*)(blockList.front()))->getBoundingBox();
    addBlocks(blockList);

    _init(map,entityName, false);
}

BlockEntity::BlockEntity(Octree3DMapManager* map,vector<BlockEntity*> subEntities, std::string entityName)
{
    addSubEntities(subEntities);
    _init(map,entityName, true);
}

BlockEntity::~BlockEntity()
{
    clearAllBlocks();
    spaceMap->newDisappearBlockEntityList.push_back(mEntityNode);

}

BlockEntity* BlockEntity::clone(Octree3DMapManager* _newSpaceMap)
{

    BlockEntity* cloneBlockEntity = new BlockEntity(_newSpaceMap,is_superBlockEntity,mFartherEntity,mID,mName,
                                                    mBoundingBox,mCenterPosition,mMyBlocks,mMySubEntities);

    return cloneBlockEntity;

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
    vector<Block3D*>::iterator iter = mMyBlocks.begin();
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

void BlockEntity::addSubEntity(BlockEntity* subEntity)
{
    mMySubEntities.push_back(subEntity);
    subEntity->setFartherEntity(this);
    _reCalculateBoundingBox();
}

void BlockEntity::addSubEntities(vector<BlockEntity*> subEntityList)
{
    vector<BlockEntity*>::iterator iter = subEntityList.begin();
    for (; iter != subEntityList.end(); ++ iter)
    {
        addSubEntity((BlockEntity*)(*iter));
    }
}

void BlockEntity::removeSubEntity(BlockEntity* subEntityToRemove)
{
    vector<BlockEntity*>::iterator iter = mMySubEntities.begin();
    for (; iter != mMySubEntities.end(); ++ iter)
    {
        if (subEntityToRemove == *iter)
        {
            subEntityToRemove->setFartherEntity(0);
            mMySubEntities.erase(iter);
            break;
        }
    }

    _reCalculateBoundingBox();
}

void BlockEntity::removeSubEntities(vector<BlockEntity*> subEntityListToRemove)
{
    vector<BlockEntity*>::iterator iter = subEntityListToRemove.begin();
    for (; iter != subEntityListToRemove.end(); ++ iter)
    {
        removeSubEntity((BlockEntity*)(*iter));
    }

}

void BlockEntity::_ReCalculatCenterPosition()
{
    mCenterPosition.x = mBoundingBox.nearLeftBottomConer.x + mBoundingBox.size_x / 2;
    mCenterPosition.y = mBoundingBox.nearLeftBottomConer.y + mBoundingBox.size_y / 2;
    mCenterPosition.z = mBoundingBox.nearLeftBottomConer.z + mBoundingBox.size_z / 2;
}

void BlockEntity::_reCalculateBoundingBox()
{
    if (mMyBlocks.size() != 0)
    {
        mBoundingBox = ((Block3D*)mMyBlocks.front())->getBoundingBox();
        vector<Block3D*>::iterator iter = mMyBlocks.begin();
        iter ++;
        for (; iter != mMyBlocks.end(); ++ iter)
        {
            mBoundingBox += ((Block3D*)(*iter))->getBoundingBox();
        }
    }

    if (mMySubEntities.size() != 0)
    {
        mBoundingBox = ((BlockEntity*)mMySubEntities.front())->getBoundingBox();
        vector<BlockEntity*>::iterator iter2 = mMySubEntities.begin();
        iter2 ++;
        for (; iter2 != mMySubEntities.end(); ++ iter2)
        {
            mBoundingBox += ((BlockEntity*)(*iter2))->getBoundingBox();
        }
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

        if (blocks[j]->getPosition().z > blocks[j+1]->getPosition().z)
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
        if (b)
        {
            if (b->mBlockEntity == this)
                b->mBlockEntity = 0;
        }
    }

    mMyBlocks.clear();
    mBoundingBox = AxisAlignedBox::ZERO;

}

// this constructor is only for clone this instance (clone it from a spaceMap to another spaceMap)
BlockEntity::BlockEntity(Octree3DMapManager* _newSpaceMap,bool _is_superBlockEntity,BlockEntity* _FartherEntity,
            int _ID, std::string _Name, AxisAlignedBox& _BoundingBox, BlockVector& _CenterPosition,vector<Block3D*>& _MyBlocks,vector<BlockEntity*>& _MySubEntities):
    spaceMap(_newSpaceMap),is_superBlockEntity(_is_superBlockEntity),mFartherEntity(_FartherEntity)
{
    mID = _ID;
    mName = _Name;
    mBoundingBox = _BoundingBox;
    mCenterPosition = _CenterPosition;

    mEntityClass = "block";
    mYaw = 0.0f; // currently, we don't consider about the rotation of block entities

    // blocks are always obstacle
    is_obstacle = true;

    vector<Block3D*>::iterator it;
    for (it = _MyBlocks.begin(); it != _MyBlocks.end(); ++it)
    {
        Block3D* b = *it;
        Block3D* blockInNewMap;
        _newSpaceMap->getRootOctree()->checkIsSolid( b->getPosition(), blockInNewMap);
        if (blockInNewMap != 0)
        {
            mMyBlocks.push_back(blockInNewMap);
            blockInNewMap->mBlockEntity = this;
        }
        else
        {
            logger().error("BlockEntity::clone: Cannot find this block (%d,%d,%d) in new space map!/n Please Make sure you have clone the Octrees before clone the BlockEntities!/n",
                           b->getPosition().x,b->getPosition().y,b->getPosition().z);
        }
    }

    if (is_superBlockEntity)
    {
        //TODO:
    }

}
