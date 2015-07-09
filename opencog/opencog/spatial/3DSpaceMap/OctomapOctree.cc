#include <algorithm>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <opencog/util/Logger.h>


#include "Block3D.h"
#include "Block3DMapUtil.h"
#include "OctomapOctree.h"

using namespace opencog;
using namespace opencog::spatial;
using namespace octomap;

void OctomapOcTreeNode::cloneNodeRecur(const OctomapOcTreeNode& rhs)
{
	if(rhs._block==NULL)
	{
		this->_block=NULL;
	}
	else
	{
		this->_block = (rhs._block)->clone();
	}

    if (rhs.hasChildren())
	{
		for (unsigned i = 0; i<8; ++i)
		{
			if (rhs.children[i])
			{
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

OctomapOcTree::OctomapOcTree(const OctomapOcTree& rhs):OccupancyOcTreeBase <OctomapOcTreeNode>(rhs){}

OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const double& x, const double& y,
											   const double& z, Block3D *const & block) 
{
	point3d pos(x,y,z);
	return setNodeBlock(pos, block);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const point3d& pos, Block3D *const & block) 
{
	OcTreeKey key;
	if (!this->coordToKeyChecked(pos, key)) return NULL;
	return setNodeBlock(key, block);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const OcTreeKey& key, Block3D* const & block)
{
    OctomapOcTreeNode* n = search(key);
    if (n != NULL) 
	{
		n->setBlock(block); 
    }
    return n;
}

//assume the block parameter has pointed to an block instance
//But it seems no way to check if poitner is valid.
//I think we should use smart pointer here, but that needs lots of refactor work..
void OctomapOcTree::addSolidBlock(Block3D* block)
{
	
	BlockVector pos=block->getPosition();
	OctomapOcTreeNode* blocknode=this->search(pos.x,pos.y,pos.z);
	if(blocknode!=NULL && blocknode->getBlock()!=NULL)
	{
		logger().error("There has been a block at %f,%f,%f. Can't add solid block.",pos.x,pos.y,pos.z);
		return;
	}

	this->updateNode(pos.x,pos.y,pos.z,true);
	this->setNodeBlock(pos.x,pos.y,pos.z,block);
}

bool OctomapOcTree::checkIsOutOfRange(const BlockVector& pos) const 
{
	OcTreeKey key;
	return !coordToKeyChecked(pos.x,pos.y,pos.z,key);
}

bool OctomapOcTree::checkIsSolid(const BlockVector& pos, Block3D* & block) const
{

	OctomapOcTreeNode* blocknode=this->search(pos.x,pos.y,pos.z);
	if(blocknode==NULL || blocknode->getBlock()==NULL)
	{
		block=NULL;
		return false;
	}
	else
	{
		block=blocknode->getBlock();
		return true;
	}
}


bool OctomapOcTree::removeAnUnitSolidBlock(const BlockVector& pos,unsigned depth)
{
	return this->deleteNode(pos.x,pos.y,pos.z,depth);
}

BlockVector OctomapOcTree::getNeighbourSolidBlockVector(BlockVector& pos, Block3D* &neighbourBlock)
{
    // check 26 neighbours

	for (int i = -1; i < 2; i ++)
		for (int j = -1; j < 2; j ++)
			for (int k = -1; k < 2; k ++)
                {
                    if (i == 0 && j == 0 && k == 0)
                        continue;
                    BlockVector nextPos(pos.x + i, pos.y + j, pos.z + k);
                    if ( checkIsSolid(nextPos,neighbourBlock))
						{
							return nextPos;
						}
                }
	
	return BlockVector::ZERO;
}

vector<Block3D*> OctomapOcTree::findAllBlocksCombinedWith(BlockVector* pos, bool useBlockMaterial)
{
    vector<Block3D*> blockList;
    vector<BlockVector> searchList, searchedList;
    searchList.push_back(*pos);

    // check the 26 neighbours unit blockvectors of every block combined
    // by a breadth-first searching
    BlockVector curPos;
    Block3D* curblock;
    Block3D* nextBlock;

    while(searchList.size() != 0)
    {
        curPos = searchList.front();
        if (checkIsSolid(curPos, curblock))
        {	
            for (int i = -1; i < 2; i ++)
			{
                for (int j = -1; j < 2; j ++)
				{
                    for (int k = -1; k < 2; k ++)
                    {
                        if (i == 0 && j == 0 && k == 0)
						{continue;}
                        BlockVector nextPos(curPos.x + i, curPos.y + j, curPos.z + k);	
                        // if this nextPos already searched, don't push in searchList
						if(find(searchedList.cbegin(),searchedList.cend(),nextPos)!=searchedList.cend())
						{continue;}

                        if (checkIsSolid(nextPos, nextBlock) && ( (!useBlockMaterial) || (nextBlock->getBlockMaterial() == curblock->getBlockMaterial())))
                        {
							if(find(searchList.cbegin(),searchList.cend(),nextPos)!=searchList.cend())
							{searchList.push_back(nextPos);}

                            if (nextBlock->mBlockEntity == 0 && find(blockList.cbegin(),blockList.cend(),nextBlock)!=blockList.cend())
							{blockList.push_back(nextBlock);}		
                        }
                        else {searchedList.push_back(nextPos);}
                    }
				}
			}
		}
        searchedList.push_back(curPos);
        searchList.erase(searchList.begin());
	}

    return blockList;
}

vector<BlockVector> OctomapOcTree::getAllNeighbourSolidBlockVectors(BlockVector& pos)
{
    vector<BlockVector> vectorList;

    Block3D* neighbourBlock;

    // check 26 neighbours

	for (int i = -1; i < 2; i ++)
	{
		for (int j = -1; j < 2; j ++)
		{
			for (int k = -1; k < 2; k ++)
			{
				if (i == 0 && j == 0 && k == 0){ continue;}

				BlockVector nextPos(pos.x + i, pos.y + j, pos.z + k);
				if ( checkIsSolid(nextPos,neighbourBlock))
				{ vectorList.push_back(nextPos);}
			}
		}
	}
	return vectorList;
}


vector<BlockEntity*> OctomapOcTree::getNeighbourEntities(BlockVector& pos)
{
    vector<BlockEntity*> entities;
    Block3D* beginBlock;
    checkIsSolid(pos, beginBlock);
    // check the 26 neighbours unit blockvectors of every block combined
    Block3D* block;

    for (int i = -1; i < 2; i ++)
	{
        for (int j = -1; j < 2; j ++)
		{
            for (int k = -1; k < 2; k ++)
            {
                if ((i == 0) && (j == 0) && (k == 0))
                    continue;
                BlockVector nextPos(pos.x + i,pos.y + j, pos.z + k);
                if (checkIsSolid(nextPos, block))
                {
                    // there is a block in this neighbour pos
                    if (block == beginBlock) {continue;}

                    // we don't contain the begin block in our return list
					if(find(entities.cbegin(),entities.cend(),block->mBlockEntity)==entities.cend())
					{entities.push_back(block->mBlockEntity);}
                }
            }
		}
	}
    return entities;
}
