#include <algorithm>

#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <opencog/util/exceptions.h>
#include "OpencogOcTree.h"

using namespace opencog;
using namespace opencog::spatial;
using namespace octomap;

//helper
point3d blockVectorToPoint3d(BlockVector pos)
{
    return point3d(pos.x, pos.y, pos.z);
}

BlockVector point3dToBlockVector(point3d pos)
{
    return BlockVector(pos.x(), pos.y(), pos.z());
}

void OpencogOcTreeNode::cloneNodeRecur(const OpencogOcTreeNode& rhs)
{
    mblockHandle = rhs.mblockHandle;
    if (rhs.hasChildren()) {
        for (unsigned i = 0; i<8; ++i) {
            if (rhs.children[i]) {
                OpencogOcTreeNode* thisChild = static_cast<OpencogOcTreeNode*>(this->children[i]);
                thisChild->cloneNodeRecur(*(static_cast<OpencogOcTreeNode*>(rhs.children[i])));
            }
        }
    }

}

OpencogOcTreeNode::OpencogOcTreeNode(const OpencogOcTreeNode& rhs): OcTreeNode(rhs)
{
    this->cloneNodeRecur(rhs);
}

OpencogOcTreeNode& OpencogOcTreeNode::operator=(const OpencogOcTreeNode& rhs)
{
    this->cloneNodeRecur(rhs);
    return *this;
}

OpencogOcTreeNode* OpencogOcTree::setNodeBlock(const double& x,
                                               const double& y,
                                               const double& z,
                                               const Handle& block)
{
    point3d pos(x, y, z);
    return setNodeBlock(pos, block);
}


OpencogOcTreeNode* OpencogOcTree::setNodeBlock(const point3d& pos, const Handle& block)
{
    OpencogOcTreeNode* n = search(pos);
    if (n != NULL) {
        // add/remove record in atom->position map
        Handle oldBlock = n->getBlock();
        if (oldBlock == Handle::UNDEFINED && block != Handle::UNDEFINED) {
            mTotalUnitBlockNum++;
            mAllUnitAtomsToBlocksMap.insert(pair<Handle, BlockVector>(block, point3dToBlockVector(pos)));
        } else if (oldBlock != Handle::UNDEFINED && block == Handle::UNDEFINED) {
            mTotalUnitBlockNum--;
            mAllUnitAtomsToBlocksMap.erase(oldBlock);
        }

        n->setBlock(block);
    }

    return n;
}

Handle OpencogOcTree::getBlock(const BlockVector& pos) const
{
    return getBlock(pos, occ_prob_thres_log);
}

Handle OpencogOcTree::getBlock(const BlockVector& pos, const float logOddsOccupancyThreshold) const
{
    OpencogOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsOccupancyThreshold) {
        return Handle::UNDEFINED;
    } else {
        return blocknode->getBlock();
    }
}


bool OpencogOcTree::checkIsOutOfRange(const BlockVector& pos) const
{
    OcTreeKey key;
    return !coordToKeyChecked(pos.x, pos.y, pos.z, key);
}

bool OpencogOcTree::checkBlockInPos(const Handle& block, const BlockVector& pos) const
{
    return checkBlockInPos(block, pos, occ_prob_thres_log);
}


bool OpencogOcTree::checkBlockInPos(const Handle& block,
                                    const BlockVector& pos,
                                    const float logOddsOccupancyThreshold) const
{
    OpencogOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsOccupancyThreshold ||
        blocknode->getBlock() != block) {
        return false;
    } else {
        return true;
    }
}

OpencogOcTree::OpencogOcTree(const std::string& mapName,const double resolution):
    OccupancyOcTreeBase<OpencogOcTreeNode>(resolution),
    mMapName(mapName)
{
    //set default agent height as 1
    mAgentHeight = 1;
}

OpencogOcTree* OpencogOcTree::clone() const
{
    OpencogOcTree* cloneMap = new OpencogOcTree(*this);
    return cloneMap;
}


void OpencogOcTree::addSolidUnitBlock(const Handle& block, BlockVector pos)
{
    setUnitBlock(block, pos, getOccupancyThresLog());
}


void OpencogOcTree::removeSolidUnitBlock(const Handle blockHandle)
{
    auto it = mAllUnitAtomsToBlocksMap.find(blockHandle);
    if (mAllUnitAtomsToBlocksMap.end() == it)
        throw opencog::NotFoundException(TRACE_INFO,
             "OpencogOcTree::removeSolidUnitBlock");

    BlockVector pos = it->second;
    OpencogOcTreeNode* n = search(pos.x, pos.y, pos.z);
    if (NULL == n)
        throw opencog::NotFoundException(TRACE_INFO,
             "OpencogOcTree::removeSolidUnitBlock");

    float curLogOdds = n->getLogOdds();
    float thres = getOccupancyThresLog();
    if (thres > curLogOdds) {
        // the occupancy of the block is smaller than threshold,
        // so we've regard it as freespace.
        return;
    } else {
        // reduce its log odds to thres - prob_miss_log, so we can
        // regard it as freespace.
        float updatedLogOdds = thres - curLogOdds - prob_miss_log;
        setUnitBlock(Handle::UNDEFINED, pos, updatedLogOdds);
    }
}


void OpencogOcTree::setUnitBlock(const Handle& block, BlockVector pos, float updateLogOddsOccupancy)
{
    this->updateNode(pos.x, pos.y, pos.z, float(updateLogOddsOccupancy));
    this->setNodeBlock(pos.x, pos.y, pos.z, block);
}

BlockVector OpencogOcTree::getBlockLocation(const Handle& block) const
{
    return getBlockLocation(block, this->getOccupancyThresLog());
}

BlockVector OpencogOcTree::getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const
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

OpencogOcTree::OpencogOcTree(const OpencogOcTree& rhs):
    OccupancyOcTreeBase <OpencogOcTreeNode>(rhs),
    mMapName(rhs.mMapName),
    mAgentHeight(rhs.mAgentHeight),
    mAllUnitAtomsToBlocksMap(rhs.mAllUnitAtomsToBlocksMap)
{

}
