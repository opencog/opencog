/*
 * opencog/spatial/3DSpaceMap/Octree3DMapManager.cc
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


#include <algorithm>
#include <iterator>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

// #ifdef HAVE_PROTOBUF
// #include <opencog/embodiment/Learning/LearningSpace/LearningMessage.info.pb.h>
// #endif
#include "Octree3DMapManager.h"
#include "SpaceMapUtil.h"
using namespace opencog;
using namespace opencog::spatial;
/*
Octree3DMapManager::Octree3DMapManager(AtomSpace* atomspace, const std::string& mapName,const unsigned& resolution, const int floorHeight, const float agentHeight):
    mAtomSpace(atomspace), mMapName(mapName), mFloorHeight(floorHeight), mAgentHeight(agentHeight)
{

    mOctomapOctree = new OctomapOcTree(resolution);
    mAllUnitAtomsToBlocksMap.clear();
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    nonBlockEntitieshistoryLocations.clear();
    mSelfAgentEntity = Handle::UNDEFINED;
}

Octree3DMapManager::~Octree3DMapManager()
{
    // delete octree
    delete mOctomapOctree;
}

Octree3DMapManager* Octree3DMapManager::clone()
{
    Octree3DMapManager* cloneMap = new Octree3DMapManager(enable_BlockEntity_Segmentation,mMapName, mOctomapOctree,mFloorHeight,mAgentHeight,mTotalUnitBlockNum,mSelfAgentEntity, mAtomSpace, mAllUnitAtomsToBlocksMap, mAllNoneBlockEntities, mPosToNoneBlockEntityMap, mAllAvatarList, nonBlockEntitieshistoryLocations);
    return cloneMap;
}

void Octree3DMapManager::setLogOddsOccupiedThreshold(float logOddsOccupancy)
{
    mOctomapOctree->setOccupancyThres(logOddsOccupancy);
}

BlockVector Octree3DMapManager::getKnownSpaceMinCoord() const
{
    BlockVector coord;
    mOctomapOctree->getMetricMin(coord.x,coord.y,coord.z);
    return coord;
}
BlockVector Octree3DMapManager::getKnownSpaceMaxCoord() const
{
    BlockVector coord;
    mOctomapOctree->getMetricMax(coord.x,coord.y,coord.z);
    return coord;
}
BlockVector Octree3DMapManager::getKnownSpaceDim() const
{
    BlockVector dim;
    mOctomapOctree->getMetricSize(dim.x,dim.y,dim.z);
    return dim;
}


void Octree3DMapManager::addSolidUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos)
{
    float occupiedthres = mOctomapOctree->getOccupancyThresLog();
    setUnitBlock(_unitBlockAtom,_pos,occupiedthres);
}


void Octree3DMapManager::removeSolidUnitBlock(const Handle blockHandle)
{
    map<Handle, BlockVector>::iterator it;
    it = mAllUnitAtomsToBlocksMap.find(blockHandle);
    if(it == mAllUnitAtomsToBlocksMap.end()) {
        logger().error("Octree3DMapManager::removeSolidUnitBlock: Cannot find this unit block in space map!/n");
    }

    BlockVector pos = it->second;
    addSolidUnitBlock(Handle::UNDEFINED, pos);
}


void Octree3DMapManager::setUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos, float updateLogOddsOccupancy)
{
    if(mOctomapOctree->checkIsOutOfRange(_pos)) {
        logger().error("addSolidUnitBlock: You want to add a unit block which outside the limit of the map: at x = %f, y = %f, z= %f ! /n",
                       _pos.x,_pos.y,_pos.z);
        return;
    }
    Handle oldBlock = mOctomapOctree->getBlock(_pos);

    if(oldBlock == Handle::UNDEFINED && _unitBlockAtom != Handle::UNDEFINED)
    {
        mTotalUnitBlockNum++;
        mAllUnitAtomsToBlocksMap.insert(pair<Handle, BlockVector>(_unitBlockAtom, _pos));

    }
    else if(oldBlock != Handle::UNDEFINED && _unitBlockAtom == Handle::UNDEFINED)
    {
        mTotalUnitBlockNum--;
        mAllUnitAtomsToBlocksMap.erase(oldBlock);
    }
    mOctomapOctree->setBlock(_unitBlockAtom, _pos, updateLogOddsOccupancy);
}

bool Octree3DMapManager::checkStandable(const BlockVector& pos) const
{
    return checkStandable(pos,mOctomapOctree->getOccupancyThresLog());
}

bool Octree3DMapManager::checkStandable(const BlockVector &pos, float logOddsOccupancy) const
{
    if(mOctomapOctree->checkIsOutOfRange(pos))
    {
        logger().error("checkstandable: You want to add a unit block which outside the limit of the map: at x = %d, y = %d, z= %d ! /n",
                       pos.x,pos.y,pos.z);
        return false;
    }


    // check if there is any non-block obstacle in this pos
    Handle blockHandle = mOctomapOctree->getBlock(pos,logOddsOccupancy);
    if(blockHandle != Handle::UNDEFINED) {
        return false;
    }
    if(pos.z <= mFloorHeight) {
        return false;
    }
    // because the agent has a height,
    // we should check if there are enough room above this pos for this agent to stand on
    if(mAgentHeight > 1) {
        for (int height = 1; height < mAgentHeight; height ++) {
            BlockVector blockAbove(pos.x, pos.y, pos.z + height);
            if(mOctomapOctree->getBlock(blockAbove, logOddsOccupancy) != Handle::UNDEFINED){
                logger().error("in %f %f %f is not undef", pos.x, pos.y, pos.z + height);
                return false;
            }
        }
    }

    // because there are too many blocks for the whole floor, we don't send these floor blocks to opencog
    // so we just check if the pos height is just on the floor.
    if( (pos.z - mFloorHeight) == 1) {
        return true;
    }
    BlockVector under(pos.x, pos.y, pos.z - 1);
    Handle underBlock = mOctomapOctree->getBlock(under, logOddsOccupancy);
    if(underBlock != Handle::UNDEFINED) {
        //TODO:Judge if this block is standable
        //if agent can't stand on it (ex.water/lava)return false
        HandleSeq blocks;
        blocks.push_back(underBlock);
        vector<string> materialPredicates = getPredicate(*mAtomSpace, "material", blocks, 1);
        if(materialPredicates.empty()){
            logger().error("checkStandable - underBlock is not undefined but no material predicate!");
            return false;
        }
        string materialOfUnderBlock = materialPredicates[0];
        if(materialOfUnderBlock == "water") {
            return false;
        } else {
            return true;
        }
    }

    return false;
}


Handle Octree3DMapManager::getBlock(const BlockVector& pos) const
{
    return mOctomapOctree->getBlock(pos);
}

Handle Octree3DMapManager::getBlock(const BlockVector& pos, float logOddsOccupancy) const
{
    return mOctomapOctree->getBlock(pos, logOddsOccupancy);
}

BlockVector Octree3DMapManager::getBlockLocation(const Handle& block) const
{
    return getBlockLocation(block, mOctomapOctree->getOccupancyThresLog());
}
BlockVector Octree3DMapManager::getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const
{
    auto it = mAllUnitAtomsToBlocksMap.find(block);
    if(it == mAllUnitAtomsToBlocksMap.end()) {
        return BlockVector::ZERO;
    } else {
        BlockVector result=it->second;
        if(getBlockLogOddsOccupancy(result)<logOddsOccupancyThreshold) {
            return BlockVector::ZERO;
        } else {
            return result;
        }
    }
}

float Octree3DMapManager::getBlockLogOddsOccupancy(const BlockVector& pos) const
{
    return mOctomapOctree->search(pos.x,pos.y,pos.z)->getLogOdds();
}

// currently we consider all the none block entities has no collision, agents can get through them
void Octree3DMapManager::addNoneBlockEntity(const Handle &entityNode, const BlockVector& pos,bool isSelfObject, bool isAvatarEntity, const unsigned long timestamp)
{
    auto it = mAllNoneBlockEntities.find(entityNode);
    if(it == mAllNoneBlockEntities.end()) {
        mAllNoneBlockEntities.insert(entityNode);
        mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(pos,entityNode));
        if(isSelfObject){ mSelfAgentEntity = entityNode;}
        if(isAvatarEntity){ mAllAvatarList.insert(entityNode);}
        _addNonBlockEntityHistoryLocation(entityNode,pos, timestamp);
    }
    else{ updateNoneBlockEntityLocation(entityNode,pos,timestamp);}
}

void Octree3DMapManager::_addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp)
{
    auto it = nonBlockEntitieshistoryLocations.find(entityHandle);
    if(it == nonBlockEntitieshistoryLocations.end())
    {
        vector< pair< unsigned long,BlockVector> > newVector;
        newVector.push_back(pair<unsigned long,BlockVector>(timestamp,newLocation));
        nonBlockEntitieshistoryLocations.insert(map<Handle, vector< pair<unsigned long, BlockVector> > >::value_type(entityHandle, newVector));
    } else {
        vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if(newLocation == (oneEntityHistories.back()).second){
            return;
        } else {
            oneEntityHistories.push_back(pair<unsigned long,BlockVector>(timestamp,newLocation));
        }
    }
}
// currently we consider all the none block entities has no collision, agents can get through them

void Octree3DMapManager::removeNoneBlockEntity(const Handle& entityNode)
{
    auto it = mAllNoneBlockEntities.find(entityNode);
    if(it != mAllNoneBlockEntities.end()) {
        // delete from mPosToNoneBlockEntityMap first
        BlockVector pos = getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(pos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(pos);
        while(biter != eiter) {
            if(biter->second == entityNode) {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
        mAllNoneBlockEntities.erase(it);
    }
}

void Octree3DMapManager::updateNoneBlockEntityLocation(const Handle &entityNode, BlockVector newpos, unsigned long timestamp)
{
    mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(newpos, entityNode));

    if(mAllNoneBlockEntities.find(entityNode) != mAllNoneBlockEntities.end()) {
        BlockVector lastpos=getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(lastpos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(lastpos);
        while(biter != eiter) {
            if(biter->second == entityNode) {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
    }
    _addNonBlockEntityHistoryLocation(entityNode, newpos, timestamp);
}

BlockVector Octree3DMapManager::getLastAppearedLocation(const Handle& entityHandle) const
{
    auto it = nonBlockEntitieshistoryLocations.find(entityHandle);
    if(it == nonBlockEntitieshistoryLocations.end()) {
        return BlockVector::ZERO;
    } else {
        const vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if(oneEntityHistories.size() == 0) {
            return BlockVector::ZERO;
        } else {
            return (oneEntityHistories.back()).second;
        }
    }
}

Handle Octree3DMapManager::getEntity(const BlockVector& pos) const
{
    auto it = mPosToNoneBlockEntityMap.find(pos);
    if(it == mPosToNoneBlockEntityMap.end()){
        return Handle::UNDEFINED;
    } else {
        return it->second;
    }
}

// this constructor is only used for clone

Octree3DMapManager::Octree3DMapManager(bool _enable_BlockEntity_Segmentation,
                                       string _MapName, OctomapOcTree* _OctomapOctree,
                                       int _FloorHeight,
                                       int _AgentHeight,int _TotalUnitBlockNum,
                                       Handle _mSelfAgentEntity,
                                       AtomSpace* _AtomSpace,
                                       const map<Handle, BlockVector> &_AllUnitAtomsToBlocksMap,
                                       const set<Handle>& _AllNoneBlockEntities,
                                       const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
                                       const set<Handle>& _AllAvatarList,
                                       const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations):
enable_BlockEntity_Segmentation(_enable_BlockEntity_Segmentation), mMapName(_MapName),mFloorHeight(_FloorHeight),
    mAgentHeight(_AgentHeight),mTotalUnitBlockNum(_TotalUnitBlockNum), mSelfAgentEntity(_mSelfAgentEntity)
{
    // the clone order should not be change here:
    // should always clone the octree before the entity list
    mOctomapOctree = new OctomapOcTree(*_OctomapOctree);
    // copy all unit blocks
    mAllUnitAtomsToBlocksMap = _AllUnitAtomsToBlocksMap;
    mAtomSpace = _AtomSpace;
    // copy all the NoneBlockEntities and mPosToNoneBlockEntityMap and mAllAvatarList
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    mAllAvatarList.clear();
    mAllNoneBlockEntities = _AllNoneBlockEntities;
    mPosToNoneBlockEntityMap = _PosToNoneBlockEntityMap;
    mAllAvatarList = _AllAvatarList;
    nonBlockEntitieshistoryLocations = _nonBlockEntitieshistoryLocations;
}

*/
