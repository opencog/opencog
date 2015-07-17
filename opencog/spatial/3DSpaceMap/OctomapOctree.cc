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
											   const double& z, const Handle& blockHandle) 
{
	point3d pos(x,y,z);
	return setNodeBlock(pos, blockHandle);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const point3d& pos, const Handle& blockHandle) 
{
	OcTreeKey key;
	if (!this->coordToKeyChecked(pos, key)) return NULL;
	return setNodeBlock(key, blockHandle);
}


OctomapOcTreeNode* OctomapOcTree::setNodeBlock(const OcTreeKey& key, const Handle& blockHandle)
{
    OctomapOcTreeNode* n = search(key);
    if (n != NULL) 
	{
		n->setBlock(blockHandle); 
    }
    return n;
}

//assume the block parameter has pointed to an block instance
//But it seems no way to check if poitner is valid.
//I think we should use smart pointer here, but that needs lots of refactor work..
void OctomapOcTree::addSolidBlock(const Handle& blockHandle, const BlockVector& pos)
{
	
	OctomapOcTreeNode* blocknode=this->search(pos.x,pos.y,pos.z);
	if(blocknode!=NULL && blocknode->getBlock()!=Handle::UNDEFINED)
	{
		logger().error("There has been a block at %f,%f,%f. Can't add solid block.",pos.x,pos.y,pos.z);
		return;
	}
	
	this->updateNode(pos.x,pos.y,pos.z,true);
	this->setNodeBlock(pos.x,pos.y,pos.z,blockHandle);
}

bool OctomapOcTree::checkIsOutOfRange(const BlockVector& pos) const 
{
	OcTreeKey key;
	return !coordToKeyChecked(pos.x,pos.y,pos.z,key);
}

bool OctomapOcTree::checkIsSolid(const BlockVector& pos, Handle& blockHandle) const
{
	OctomapOcTreeNode* blocknode=this->search(pos.x,pos.y,pos.z);
	if(blocknode==NULL || blocknode->getBlock()==NULL)
	{
		blockHandle=Handle::UNDEFINED;
		return false;
	}
	else
	{
		blockHandle=blocknode->getBlock();
		return true;
	}
}

bool OctomapOcTree::checkBlockInPos(const BlockVector& pos, const Handle& block) const
{
	OctomapOcTreeNode* blocknode=this->search(pos.x,pos.y,pos.z);
	if(blocknode==NULL || blocknode->getBlock()!=block){ return false;}
	else{ return true;}
}

bool OctomapOcTree::removeAnUnitSolidBlock(const BlockVector& pos,unsigned depth)
{
	return this->deleteNode(pos.x,pos.y,pos.z,depth);
}

/*

// Comment on 20150713 by Yi-Shan,
// The following is old public functions about BlockEntity add/remove/query
// Because the BlockEntity feature has not designed well, 
// so we comment out all the code related to BlockEntity
// Once we need to use it/decide to do it, maybe we'll need the legacy code.


BlockVector OctomapOcTree::getNeighbourSolidBlockVector(const BlockVector& pos, Handle& neighbourBlock)
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

HandleSeq OctomapOcTree::findAllBlocksCombinedWith(AtomSpace& atomspace, BlockVector* pos, bool useBlockMaterial)
{
    HandleSeq blockList;
    vector<BlockVector> searchList, searchedList;
    searchList.push_back(*pos);

    // check the 26 neighbours unit blockvectors of every block combined
    // by a breadth-first searching
    BlockVector curPos;
    Handle curblock;
    Handle nextBlock;

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

                        if (checkIsSolid(nextPos, nextBlock) && 
							((!useBlockMaterial) || 
							 getPredicateValue(atomspace,MATERIAL_PREDICATE,nextblock) == getPredicateValue(atomspace,MATERIAL_PREDICATE,curblock)))
						{
							if(find(searchList.cbegin(),searchList.cend(),nextPos)!=searchList.cend())
							{searchList.push_back(nextPos);}

							if (getBlockEntity(nextBlock) == Handle::UNDEFINED && find(blockList.cbegin(),blockList.cend(),nextBlock)!=blockList.cend())
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

    Handle neighbourBlock;

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


HandleSeq OctomapOcTree::getNeighbourEntities(BlockVector& pos,const AtomSpace& atomspace)
{
    HandleSeq entities;
    Handle beginBlock;
    checkIsSolid(pos, beginBlock);
    // check the 26 neighbours unit blockvectors of every block combined
    Handle block;

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
                    // we don't contain the begin block in our return list
                    if (block == beginBlock) {continue;}
					Handle theBlockEntity=getBlockEntity(block);
					if (theBlockEntity!=Handle::UNDEFINED &&
						find(entities.cbegin(),entities.cend(),theBlockEntity)==entities.cend())
					{entities.push_back(theBlockEntity);}
                }
            }
		}
	}
    return entities;
}
*/
