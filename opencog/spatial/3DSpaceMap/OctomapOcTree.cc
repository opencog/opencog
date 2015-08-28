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
                OctomapOcTreeNode* thisChild = static_cast<OctomapOcTreeNode*>(this->children[i]);
                thisChild->cloneNodeRecur(*(static_cast<OctomapOcTreeNode*>(rhs.children[i])));
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

OctomapOcTree::OctomapOcTree(const std::string& mapName,const double resolution, const float agentHeight):
    OccupancyOcTreeBase<OctomapOcTreeNode>(resolution),
    mMapName(mapName), mAgentHeight(agentHeight)
{

}

OctomapOcTree* OctomapOcTree::clone()
{
    OctomapOcTree* cloneMap = new OctomapOcTree(*this);
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
    if (this->checkIsOutOfRange(_pos)) {
        logger().error("addSolidUnitBlock: You want to add a unit block which outside the limit of the map: at x = %f, y = %f, z= %f ! /n",
                       _pos.x,_pos.y,_pos.z);
        return;
    }
    Handle oldBlock = this->getBlock(_pos);

    if (oldBlock == Handle::UNDEFINED && _unitBlockAtom != Handle::UNDEFINED) {
        mTotalUnitBlockNum++;
        mAllUnitAtomsToBlocksMap.insert(pair<Handle, BlockVector>(_unitBlockAtom, _pos));
    } else if (oldBlock != Handle::UNDEFINED && _unitBlockAtom == Handle::UNDEFINED) {
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


// this constructor is only used for clone

OctomapOcTree::OctomapOcTree(const OctomapOcTree& rhs):
    OccupancyOcTreeBase <OctomapOcTreeNode>(rhs),
    mMapName(rhs.mMapName),
    mAgentHeight(rhs.mAgentHeight),
    mAllUnitAtomsToBlocksMap(rhs.mAllUnitAtomsToBlocksMap)
{

}
