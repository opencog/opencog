/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPENCOG_OCTOMAP_OCTREE_H
#define OPENCOG_OCTOMAP_OCTREE_H

#include <vector>
#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <opencog/spatial/3DSpaceMap/Block3D.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>


using namespace octomap;

namespace opencog
{
	namespace spatial 
	{
  
		class OctomapOcTreeNode : public OcTreeNode
		{
		public:
		    OctomapOcTreeNode() : OcTreeNode(), _block(NULL) {}
			OctomapOcTreeNode(const OctomapOcTreeNode& rhs);
			~OctomapOcTreeNode(){ delete _block;}
			OctomapOcTreeNode& operator=(const OctomapOcTreeNode& rhs);
			// children
			inline OctomapOcTreeNode* getChild(unsigned int i) 
			{
				return static_cast<OctomapOcTreeNode*> (OcTreeNode::getChild(i));
			}
			inline const OctomapOcTreeNode* getChild(unsigned int i) const 
			{
				return static_cast<const OctomapOcTreeNode*> (OcTreeNode::getChild(i));
			}

			bool createChild(unsigned int i) 
			{
				if (children == NULL) allocChildren();
				children[i] = new OctomapOcTreeNode();
				return true;
			}

			void setBlock(Block3D *const & block)
			{
				_block=block;
			}

			const Block3D* getBlock() const
			{
				return _block;
			}

			Block3D* getBlock()
			{
				return _block;
			}

			void cloneNodeRecur(const OctomapOcTreeNode& rhs);
		private:
			Block3D* _block;
		};

		// tree definition
		class OctomapOcTree : public OccupancyOcTreeBase <OctomapOcTreeNode> {
			
		public:
			/// Default constructor, sets resolution of leafs
		OctomapOcTree(double resolution) : OccupancyOcTreeBase<OctomapOcTreeNode>(resolution) {}
			OctomapOcTree(const OctomapOcTree&);
			
			/// virtual constructor: creates a new object of same type
			/// (Covariant return type requires an up-to-date compiler)
			OctomapOcTree* create() const {return new OctomapOcTree(resolution); }
			
			std::string getTreeType() const {return "OctomapOcTree";}
   
			// set node color at given key or coordinate. Replaces previous color.
			OctomapOcTreeNode* setNodeBlock(const OcTreeKey& key, Block3D *const & block);
			OctomapOcTreeNode* setNodeBlock(const double& x, const double& y,
											const double& z, Block3D *const & block); 

			OctomapOcTreeNode* setNodeBlock(const point3d& pos, Block3D *const & block);

			// Move the Opencog Octree legacy API here

			// Add a block.This block is not necessary to be a unit block, it can be a bigger block
			void addSolidBlock(Block3D * block);
		
			//  Remove an unit block at a given position from the octree system
			//  return false if block not exists.
			bool removeAnUnitSolidBlock(const BlockVector& pos, unsigned depth=0);
			
			// Check whether this position has been filled by a solid block,
			// Note that if the block is in the unknown space it'll also return false
			// pos is input para, block is output para
			// if solid, return the block in @ block
			bool checkIsSolid(const BlockVector& pos, Block3D* & block) const;


			// find all the blocks combine with the block in pos,
			// the return list does not contain the block in this pos
			vector<Block3D*>  findAllBlocksCombinedWith(BlockVector* pos, bool useBlockMaterial = true);

			// get all the existing BlockEntities will combined by this block (if add a block in this pos)
			// calculate all the 26 neighbours
			vector<BlockEntity*> getNeighbourEntities(BlockVector& pos);
		
			// get any a Neighbour Solid BlockVector of curPos
			// the neighbourBlock will return the block contains this neighbour BlockVector
			BlockVector getNeighbourSolidBlockVector(BlockVector& pos , Block3D* &neighbourBlock);

			// return all the neighbour sold BlockVector of curPos
			vector<BlockVector> getAllNeighbourSolidBlockVectors(BlockVector& pos);
    
		protected:

			/**
			 * Static member object which ensures that this OcTree's prototype
			 * ends up in the classIDMapping only once
			 */
			class StaticMemberInitializer{
			public:
				StaticMemberInitializer() {
					OctomapOcTree* tree = new OctomapOcTree(0.1);
					AbstractOcTree::registerTreeType(tree);
				}
			};
			// static member to ensure static initialization (only once)
			static StaticMemberInitializer colorOcTreeMemberInit;
		};

	} // end namespace
}
#endif
