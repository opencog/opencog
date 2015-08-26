#include <algorithm>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <opencog/util/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include "Block3DMapUtil.h"
#include "OctomapOcTree.h"

/***Working now to move Octreemanager!!!***/
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

/***Working now to move Octreemanager!!!***/


using namespace opencog;
using namespace opencog::spatial;
using namespace octomap;

void OctomapOcTreeNode::cloneNodeRecur(const OctomapOcTreeNode& rhs)
{
    mblockHandle=rhs.mblockHandle;
    if (rhs.hasChildren()) {
        for (unsigned i = 0; i<8; ++i) {
            if (rhs.children[i]) {
                (static_cast<OctomapOcTreeNode*>(this->children[i]))->cloneNodeRecur(*(static_cast<OctomapOcTreeNode*>(rhs.children[i])));
            }
        }
    }

}

OctomapOcTreeNode::OctomapOcTreeNode(const OctomapOcTreeNode& rhs): OcTreeNode(rhs)
{
    this->cloneNodeRecur(rhs);
}

OctomapOcTreeNode& OctomapOcTreeNode::operator=(const OctomapOcTreeNode& rhs)
{
    this->cloneNodeRecur(rhs);
    return *this;
}

OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const double& x,
                                               const double& y,
                                               const double& z,
                                               const Handle& block)
{
    point3d pos(x, y, z);
    return setNodeBlock(pos, block);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const point3d& pos, const Handle& block)
{
    OcTreeKey key;
    if (!this->coordToKeyChecked(pos, key)) {
        return NULL;
    }
    return setNodeBlock(key, block);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const OcTreeKey& key, const Handle& block)
{
    OctomapOcTreeNode* n = search(key);
    if (n != NULL) {
        n->setBlock(block);
    }
    return n;
}


void OctomapOcTree::setBlock(const Handle& block,
                             const BlockVector& pos,
                             const bool isOccupied)
{
    OctomapOcTreeNode* blockNode;
    if (isOccupied) {
        blockNode = this->updateNode(pos.x, pos.y, pos.z, prob_hit_log);
    } else {
        blockNode = this->updateNode(pos.x, pos.y, pos.z, -prob_hit_log-0.1f);
    }
    blockNode->setBlock(block);
}
void OctomapOcTree::setBlock(const Handle& block,
                             const BlockVector& pos,
                             const float logOddsOccupancyUpdate)
{
    this->updateNode(pos.x, pos.y, pos.z, float(logOddsOccupancyUpdate));
    this->setNodeBlock(pos.x, pos.y, pos.z, block);
}

Handle OctomapOcTree::getBlock(const BlockVector& pos) const
{
    return getBlock(pos, occ_prob_thres_log);
}

Handle OctomapOcTree::getBlock(const BlockVector& pos, const float logOddsThreshold) const
{
    OctomapOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsThreshold) {
        return Handle::UNDEFINED;
    } else {
        return blocknode->getBlock();
    }
}


bool OctomapOcTree::checkIsOutOfRange(const BlockVector& pos) const
{
    OcTreeKey key;
    return !coordToKeyChecked(pos.x, pos.y, pos.z, key);
}

bool OctomapOcTree::checkBlockInPos(const Handle& block, const BlockVector& pos) const
{
    return checkBlockInPos(block, pos, occ_prob_thres_log);
}


bool OctomapOcTree::checkBlockInPos(const Handle& block,
                                    const BlockVector& pos,
                                    const float logOddsThreshold) const
{
    OctomapOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsThreshold ||
        blocknode->getBlock() != block) {
        return false;
    } else {
        return true;
    }
}




        /************
              Working Now to move the interface of Octree3DMapManger!!
        *************/





OctomapOcTree::OctomapOcTree(AtomSpace* atomspace, const std::string& mapName,const double resolution, const int floorHeight, const float agentHeight):
    OccupancyOcTreeBase<OctomapOcTreeNode>(resolution),
    mAtomSpace(atomspace), mMapName(mapName), mFloorHeight(floorHeight), mAgentHeight(agentHeight), mSelfAgentEntity(Handle::UNDEFINED)
{
    mAllUnitAtomsToBlocksMap.clear();
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    mNoneBlockEntitieshistoryLocations.clear();
}

OctomapOcTree* OctomapOcTree::clone()
{
    OctomapOcTree* cloneMap = new OctomapOcTree(*this);
    //OctomapOcTree* cloneMap = new OctomapOcTree(mMapName, mFloorHeight,mAgentHeight, mTotalUnitBlockNum, mSelfAgentEntity, mAtomSpace, mAllUnitAtomsToBlocksMap, mAllNoneBlockEntities, mPosToNoneBlockEntityMap, mAllAvatarList, mNoneBlockEntitieshistoryLocations);
    return cloneMap;
}


void OctomapOcTree::addSolidUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos)
{
    float occupiedthres = this->getOccupancyThresLog();
    setUnitBlock(_unitBlockAtom,_pos, occupiedthres);
}


void OctomapOcTree::removeSolidUnitBlock(const Handle blockHandle)
{
    map<Handle, BlockVector>::iterator it;
    it = mAllUnitAtomsToBlocksMap.find(blockHandle);
    if(it == mAllUnitAtomsToBlocksMap.end()) {
        logger().error("OctomapOcTree::removeSolidUnitBlock: Cannot find this unit block in space map!/n");
    }

    BlockVector pos = it->second;
    addSolidUnitBlock(Handle::UNDEFINED, pos);
}


void OctomapOcTree::setUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos, float updateLogOddsOccupancy)
{
    if(this->checkIsOutOfRange(_pos)) {
        logger().error("addSolidUnitBlock: You want to add a unit block which outside the limit of the map: at x = %f, y = %f, z= %f ! /n",
                       _pos.x,_pos.y,_pos.z);
        return;
    }
    Handle oldBlock = this->getBlock(_pos);

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
    this->setBlock(_unitBlockAtom, _pos, updateLogOddsOccupancy);
}

BlockVector OctomapOcTree::getBlockLocation(const Handle& block) const
{
    return getBlockLocation(block, this->getOccupancyThresLog());
}

BlockVector OctomapOcTree::getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const
{
    auto it = mAllUnitAtomsToBlocksMap.find(block);
    if(it == mAllUnitAtomsToBlocksMap.end()) {
        return BlockVector::ZERO;
    } else {
        BlockVector result=it->second;
        if(search(result.x, result.y, result.z)->getLogOdds() < logOddsOccupancyThreshold) {
            return BlockVector::ZERO;
        } else {
            return result;
        }
    }
}

// currently we consider all the none block entities has no collision, agents can get through them
void OctomapOcTree::addNoneBlockEntity(const Handle &entityNode, const BlockVector& pos,bool isSelfObject, bool isAvatarEntity, const unsigned long timestamp)
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

void OctomapOcTree::_addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp)
{
    auto it = mNoneBlockEntitieshistoryLocations.find(entityHandle);
    if(it == mNoneBlockEntitieshistoryLocations.end())
    {
        vector< pair< unsigned long,BlockVector> > newVector;
        newVector.push_back(pair<unsigned long,BlockVector>(timestamp,newLocation));
        mNoneBlockEntitieshistoryLocations.insert(map<Handle, vector< pair<unsigned long, BlockVector> > >::value_type(entityHandle, newVector));
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

void OctomapOcTree::removeNoneBlockEntity(const Handle& entityNode)
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

void OctomapOcTree::updateNoneBlockEntityLocation(const Handle &entityNode, BlockVector newpos, unsigned long timestamp)
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

BlockVector OctomapOcTree::getLastAppearedLocation(const Handle& entityHandle) const
{
    auto it = mNoneBlockEntitieshistoryLocations.find(entityHandle);
    if(it == mNoneBlockEntitieshistoryLocations.end()) {
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

Handle OctomapOcTree::getEntity(const BlockVector& pos) const
{
    auto it = mPosToNoneBlockEntityMap.find(pos);
    if(it == mPosToNoneBlockEntityMap.end()){
        return Handle::UNDEFINED;
    } else {
        return it->second;
    }
}

// this constructor is only used for clone

OctomapOcTree::OctomapOcTree(const OctomapOcTree& rhs):
    OccupancyOcTreeBase <OctomapOcTreeNode>(rhs),
    mAtomSpace(rhs.mAtomSpace), mMapName(rhs.mMapName),
    mFloorHeight(rhs.mFloorHeight), mAgentHeight(rhs.mAgentHeight),
    mSelfAgentEntity(rhs.mSelfAgentEntity), mAllUnitAtomsToBlocksMap(rhs.mAllUnitAtomsToBlocksMap),
    mAllNoneBlockEntities(rhs.mAllNoneBlockEntities), mAllAvatarList(rhs.mAllAvatarList),
    mPosToNoneBlockEntityMap(rhs.mPosToNoneBlockEntityMap), mNoneBlockEntitieshistoryLocations(rhs.mNoneBlockEntitieshistoryLocations)
{
    
}

/*
OctomapOcTree::OctomapOcTree(string _MapName,
                             int _FloorHeight,
                             int _AgentHeight,int _TotalUnitBlockNum,
                             Handle _mSelfAgentEntity,
                             AtomSpace* _AtomSpace,
                             const map<Handle, BlockVector> &_AllUnitAtomsToBlocksMap,
                             const set<Handle>& _AllNoneBlockEntities,
                             const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
                             const set<Handle>& _AllAvatarList,
                             const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations):

    mMapName(_MapName),mFloorHeight(_FloorHeight),
    mAgentHeight(_AgentHeight),mTotalUnitBlockNum(_TotalUnitBlockNum), mSelfAgentEntity(_mSelfAgentEntity), mAllNoneBlockEntities(rhs.)
{
    // the clone order should not be change here:
    // should always clone the octree before the entity list
    mOctomapOcTree = new OctomapOcTree(*_OctomapOcTree);
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
        /************
              Working Now to move the interface of Octree3DMapManger!!
        *************/

