#include <algorithm>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
//#include <opencog/atomspace/Handle.h>
#include <opencog/util/Logger.h>
//#include "Block3DMapUtil.h"
#include "OctomapOcTree.h"

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

void OctomapOcTreeNode::cloneNodeRecur(const OctomapOcTreeNode& rhs)
{
    mblockHandle = rhs.mblockHandle;
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
    OctomapOcTreeNode* n = search(pos);
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

Handle OctomapOcTree::getBlock(const BlockVector& pos) const
{
    return getBlock(pos, occ_prob_thres_log);
}

Handle OctomapOcTree::getBlock(const BlockVector& pos, const float logOddsOccupancyThreshold) const
{
    OctomapOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsOccupancyThreshold) {
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
                                    const float logOddsOccupancyThreshold) const
{
    OctomapOcTreeNode* blocknode = this->search(pos.x, pos.y, pos.z);
    if (blocknode == NULL ||
        blocknode->getLogOdds() < logOddsOccupancyThreshold ||
        blocknode->getBlock() != block) {
        return false;
    } else {
        return true;
    }
}

OctomapOcTree::OctomapOcTree(const std::string& mapName,const double resolution):
    OccupancyOcTreeBase<OctomapOcTreeNode>(resolution),
    mMapName(mapName)
{
    //set default agent height as 1
    mAgentHeight = 1;
}

OctomapOcTree* OctomapOcTree::clone() const
{
    OctomapOcTree* cloneMap = new OctomapOcTree(*this);
    return cloneMap;
}


void OctomapOcTree::addSolidUnitBlock(const Handle& block, BlockVector pos)
{
    setUnitBlock(block, pos, getOccupancyThresLog());
}


void OctomapOcTree::removeSolidUnitBlock(const Handle blockHandle)
{
    auto it = mAllUnitAtomsToBlocksMap.find(blockHandle);
    if ( it == mAllUnitAtomsToBlocksMap.end()) {
        // XXX FIXME this should be a throw, not a logger message.
        logger().error("OctomapOcTree::removeSolidUnitBlock: "
                       "Cannot find this unit block in space map!/n");
    }

    BlockVector pos = it->second;
    OctomapOcTreeNode* n = search(pos.x, pos.y, pos.z);
    if (NULL == n) return;

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


void OctomapOcTree::setUnitBlock(const Handle& block, BlockVector pos, float updateLogOddsOccupancy)
{
    this->updateNode(pos.x, pos.y, pos.z, float(updateLogOddsOccupancy));
    this->setNodeBlock(pos.x, pos.y, pos.z, block);
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
