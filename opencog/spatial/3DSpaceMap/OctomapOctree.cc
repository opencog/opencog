#include <algorithm>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <opencog/util/Logger.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include "Block3DMapUtil.h"
#include "OctomapOctree.h"


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

OctomapOcTree::OctomapOcTree(const OctomapOcTree& rhs):
    OccupancyOcTreeBase <OctomapOcTreeNode>(rhs){}

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
