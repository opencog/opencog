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

#ifndef OCTOMAP_COLOR_OCTREE_H
#define OCTOMAP_COLOR_OCTREE_H
#include <vector>


#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/spatial/3DSpaceMap/Block3D.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>
#include "Octree3DMapManager.h"


using namespace opencog;
using namespace opencog::spatial;

namespace octomap {
  
  // node definition
  class ColorOcTreeNode : public OcTreeNode {    
  public:
    
    class Color {
    public:
    Color() : r(255), g(255), b(255) {}
    Color(unsigned char _r, unsigned char _g, unsigned char _b) 
      : r(_r), g(_g), b(_b) {}
      inline bool operator== (const Color &other) const {
        return (r==other.r && g==other.g && b==other.b);
      }
      inline bool operator!= (const Color &other) const {
        return (r!=other.r || g!=other.g || b!=other.b);
      }
      unsigned char r, g, b;
    };

  public:
    ColorOcTreeNode() : OcTreeNode() {}

    ColorOcTreeNode(const ColorOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color) {}

    bool operator==(const ColorOcTreeNode& rhs) const{
      return (rhs.value == value && rhs.color == color);
    }
    
    // children
    inline ColorOcTreeNode* getChild(unsigned int i) {
      return static_cast<ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const ColorOcTreeNode* getChild(unsigned int i) const {
      return static_cast<const ColorOcTreeNode*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i) {
      if (children == NULL) allocChildren();
      children[i] = new ColorOcTreeNode();
      return true;
    }

    bool pruneNode();
    void expandNode();
    
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = c; }
    inline void  setColor(unsigned char r, unsigned char g, unsigned char b) {
      this->color = Color(r,g,b); 
    }

    Color& getColor() { return color; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const { 
      return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
    }

    void updateColorChildren();


    ColorOcTreeNode::Color getAverageChildColor() const;
  
    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;
    
  protected:
    Color color;
  };


  // tree definition
  class ColorOcTree : public OccupancyOcTreeBase <ColorOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    ColorOcTree(double resolution) : OccupancyOcTreeBase<ColorOcTreeNode>(resolution) {};  
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    ColorOcTree* create() const {return new ColorOcTree(resolution); }

    std::string getTreeType() const {return "ColorOcTree";}
   
    // set node color at given key or coordinate. Replaces previous color.
    ColorOcTreeNode* setNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b);

    ColorOcTreeNode* setNodeColor(const float& x, const float& y, 
                                 const float& z, const unsigned char& r, 
                                 const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return setNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    ColorOcTreeNode* averageNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                  const unsigned char& g, const unsigned char& b);
    
    ColorOcTreeNode* averageNodeColor(const float& x, const float& y, 
                                      const float& z, const unsigned char& r, 
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return averageNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    ColorOcTreeNode* integrateNodeColor(const OcTreeKey& key, const unsigned char& r, 
                                  const unsigned char& g, const unsigned char& b);
    
    ColorOcTreeNode* integrateNodeColor(const float& x, const float& y, 
                                      const float& z, const unsigned char& r, 
                                      const unsigned char& g, const unsigned char& b) {
      OcTreeKey key;
      if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
      return integrateNodeColor(key,r,g,b);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    void writeColorHistogram(std::string filename);

	//Move the Opencog Octree API here, and try to implement it without change the API unless we need the change.

            // Add a block.This block is not necessary to be a unit block, it can be a bigger block
            // Usually you don't know where to place this block, so the default bool atKnownIndexes is false.
            // Only when atKnownIndexes is true, you assign values for _x, _y and _z.
            // Not that the _x,_y_,z here are not the global position of thi block, it is the child indexes of this tree
            // Strongly recommend not to use atKnownIndexes as true, unless you know this funciton very clearly.
            void addSolidBlock( Block3D * _block,bool atKnownIndexes = false, int _x = 3, int _y = 3, int _z = 3);

            //  Remove an unit block at a given position from the octree system.
            //  return the unit block atom handle.
            Handle removeAnUnitSolidBlock(BlockVector& _pos);

            //inline const BlockVector& getNearLeftBottomPoint(){return mNearLeftBottomPoint;}

			//            inline const AxisAlignedBox& getBoundingBox(){return mBoundingBox;}

            // Check whether this position has been filled by a solid block,
            // _pos is input para, block is output para
            // if solid, return the block in @ block
            bool checkIsSolid(const BlockVector& _pos, Block3D* & _block3d) const;

            // Get the parent octree that contain this _block.
            // Note that this function is only used for finding, so make sure this _block already exists.
            // Only the first parameter is input para, others are all output paras.
            // Return the parent octree, and the indexes of this _block in the mAllMyBlocks[x][y][z] of this parent octree.
            // If this _block is contain in a bigger block, isInsideABigBlock is return truem, and this bigBlock is also returned.
            // If so, the return Octree* and the indexes are all for this bigBlock.
            // If Octree* return 0, it mean this block cannot be found.
            Octree* getParentTreeOfBlock(Block3D* _block, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D*& bigBlock);

            // Same to getParentTreeOfBlock. Just the block is a unit block. And you pass the position of it to this funciton.
            // Note that this function is only used for finding, so make sure this block already exists.
            Octree* getParentTreeOfUnitBlock(BlockVector& _pos, int& x, int& y, int& z, bool& isInsideABigBlock, Block3D*& bigBlock);

			/* unused Octree get function

             Returns the Depth of this octree, the rootOctree has a Depth 1,
               and the subTrees of rootOctree has a Depth 2 and so on...
            
			            inline int getOctreeDepth()
            {
                return mOctreeDepth;
            };

            // Note, the parent of the rootOctree is null
            inline const Octree* getParent()
            {
                return mParent;
            }

            // how many unitblocks per edge of this octree
            inline int getSize()
            {
                return mSize;
            }

			*/


            /** Calculate whether this Octree has been full with solid blocks
            */
            bool isFull();

            // Check whehter this octree has become empty.
            // If it has not any subtree or block in it, it is empty.
            bool isEmpty();

            /**  Returns the appropriate indexes for the child octree of this octree into which the box will fit.
            @remarks
            This is used by the OctreeSceneManager to determine which child to traverse next when
            finding the appropriate octree to insert the box.  Since it is a loose octree, only the
            center of the box is checked to determine the octant.
            */
            void getChildIndexes( const AxisAlignedBox & _boundingbox, int &x, int &y, int &z ) const;

            void getChildIndexes( const BlockVector & _pos, int &x, int &y, int &z ) const;

            AxisAlignedBox& getChildBoundingBoxByIndex(int x, int y, int z);

            // When this octree has been full of blocks, then merge all its blocks into a bigger block
            // This will distroy  all the blocks inside, and all the atoms of the blocks will be moved to this new big block.
            // return this bigger block.
            Block3D*  mergeAllMyBlocks();

            // break this a block into 8 smaller blocks
            // Note that when this block is an unit block, you cannot do this
            // This function is usually used when a unit block is removed from a composed block
            // x,y,z is the indexes of this block to be break in mAllMyBlocks[x][y][z];
            void breakBlockInto8Blocks(int x, int y, int z);

			
			// unused octree get function
            // return its indexes in its parent if has a parent
			/*
            inline void getIndexesInParent(int& x, int& y, int& z)
            {
                if (mParent != 0)
                {
                    x = mIndex_x;
                    y = mIndex_y;
                    z = mIndex_z;
                }
            }
			*/

            // find all the blocks combine with the block in _pos,
            // the return list does not contain the block in this _pos
            vector<Block3D*>  findAllBlocksCombinedWith(BlockVector* _pos, bool useBlockMaterial = true);

            // get all the existing BlockEntities will combined by this block (if add a block in this _pos)
            // calculate all the 26 neighbours
            vector<BlockEntity*> getNeighbourEntities(BlockVector& _pos);

            // get any a Neighbour Solid BlockVector of curPos
            // the neighbourBlock will return the block contains this neighbour BlockVector
            BlockVector getNeighbourSolidBlockVector(BlockVector& curPos , Block3D* &neighbourBlock);

            // return all the neighbour sold BlockVector of curPos
            vector<BlockVector> getAllNeighbourSolidBlockVectors(BlockVector& curPos);

            // one of the 8 parts of a octree, if is not null,
            // it is either a sub octree in mChildren or a block3d in mAllMyBlocks
            /** 3D array of children of this octree.
            @remarks
            Children are dynamically created as needed when blocks are inserted in the Octree.
            the index of the children is ordered by [x][y][z]
            */
            Octree * mChildren[2][2][2];

            // a block here only means one of the 8 blocks in this octree Depth, not a unit block.
            Block3D * mAllMyBlocks[2][2][2];

            // the Octree3DMapManager
            Octree3DMapManager* mOctree3DMapManager;

            Octree* clone(Octree3DMapManager* newOctree3DMapManager, Octree* parentTree = 0);


    
  protected:
    void updateInnerOccupancyRecurs(ColorOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           ColorOcTree* tree = new ColorOcTree(0.1);
           AbstractOcTree::registerTreeType(tree);
         }
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer colorOcTreeMemberInit;

  };

  //! user friendly output in format (r g b)
  std::ostream& operator<<(std::ostream& out, ColorOcTreeNode::Color const& c);

} // end namespace

#endif
