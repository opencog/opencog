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

#include <opencog/atomspace/Handle.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>

/***Working now to move header in Octree3dmapmanager!!***/

#include <map>
#include <set>

#ifdef HAVE_ZMQ
#include <lib/zmq/zmq.hpp>
#endif

#include <opencog/atomspace/AtomSpace.h>

#include "Block3DMapUtil.h"

/***Working now to move header in Octree3dmanager!!***/
using namespace std;
using namespace octomap;

namespace opencog
{
    namespace spatial
    {

        class OctomapOcTreeNode : public OcTreeNode
        {
        public:
            OctomapOcTreeNode() : OcTreeNode(), mblockHandle(Handle::UNDEFINED){}
            OctomapOcTreeNode(const OctomapOcTreeNode& rhs);
            ~OctomapOcTreeNode(){}
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
                if (children == NULL) {
                    allocChildren();
                }
                children[i] = new OctomapOcTreeNode();
                return true;
            }

            void setBlock(const Handle& block)
            {
                mblockHandle = block;
            }

            const Handle getBlock() const
            {
                return mblockHandle;
            }

            Handle getBlock()
            {
                return mblockHandle;
            }

            void cloneNodeRecur(const OctomapOcTreeNode& rhs);
        private:
            Handle mblockHandle;
        };

        // tree definition
        class OctomapOcTree : public OccupancyOcTreeBase <OctomapOcTreeNode> {

        public:
            /// Default constructor, sets resolution of leafs
            OctomapOcTree(AtomSpace* atomspace, const std::string& mapName, const double resolution, const int floorHeight, const float agentHeight);

            /// virtual constructor: creates a new object of same type
            /// (Covariant return type requires an up-to-date compiler)
            OctomapOcTree* create() const {return new OctomapOcTree(resolution); }

            std::string getTreeType() const {return "OctomapOcTree";}

            // set node block handle at given key or coordinate. Replaces previous block handle.
            OctomapOcTreeNode* setNodeBlock(const OcTreeKey& key, const Handle& block);
            OctomapOcTreeNode* setNodeBlock(const double& x, const double& y,const double& z, const Handle& block);
            OctomapOcTreeNode* setNodeBlock(const point3d& pos, const Handle & block);

            // If you want to express binary adding/removing block, use this.
            // @isOccupied
            void setBlock(const Handle& block, const BlockVector& pos, const bool isOccupied);

            // add logOddsOccupancyUpdate value to the block log odds value
            // and set the block handle in pos
            // Note that you can control the threshold of log odds occupancy by
            // OctomapOcTree::setOccupancyThres
            void setBlock(const Handle& block, const BlockVector& pos, const float logOddsOccupancyUpdate);

            //  check if the block is out of octree's max size
            bool checkIsOutOfRange(const BlockVector& pos) const;

            //  use occ_prob_thres_log(see octomap doc) as threshold
            Handle getBlock(const BlockVector& pos) const;
            //  get block in pos. If occupancy(log odds) larger than threshold
            //  It will return the block (including undefined handle) in pos;
            //  If smaller than threshold, it'll return Handle::UNDEFINED
            //  default threshold is the prob_hit_log which is the default
            //  octomap log odds threshold.
            Handle getBlock(const BlockVector& pos, const float logOddsThreshold) const;
            //  use occ_prob_thres_log(see octomap doc) as threshold
            bool checkBlockInPos(const Handle& block, const BlockVector& pos) const;
            //  check the block is in the position,
            //  Noth that even there's a block in that pos,
            //  if the handle is not equal it still return false.
            bool checkBlockInPos(const Handle& block, const BlockVector& pos, const float logOddsThreshold) const;

        protected:

            /**
             * Static member object which ensures that this OcTree's prototype
             * ends up in the classIDMapping only once
             */

            /***temp constructor for compiled***/
        OctomapOcTree(double resolution): OccupancyOcTreeBase<OctomapOcTreeNode>(resolution){}
            class StaticMemberInitializer{
            public:
                StaticMemberInitializer() {
                    OctomapOcTree* tree = new OctomapOcTree(0.1);
                    AbstractOcTree::registerTreeType(tree);
                }
            };
            // static member to ensure static initialization (only once)
            static StaticMemberInitializer octomapOcTreeMemberInit;





            /************
              Working Now to move the interface of Octree3DMapManger!!
            *************/

        public:
            
            ~OctomapOcTree(){}
            // deep clone this octree3DMapManager and return the new instance
            OctomapOcTree* clone();

            /**
             *   getter/setter
             */
			
            inline int getFloorHeight() const {return mFloorHeight;}
            inline string getMapName() const {return mMapName;}
            inline float getAgentHeight() const {return mAgentHeight;}
            void setAgentHeight(float _height){ mAgentHeight = _height;}
            
            inline int getTotalUnitBlockNum() const {return mTotalUnitBlockNum;}
            inline Handle getSelfAgentEntity() const {return mSelfAgentEntity;}

            /**
             *  public member functions about Block add/remove/query
             */

            //binary add/remove operation
            void addSolidUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos);
            void removeSolidUnitBlock(const Handle blockHandle);
            //Note that if you want to add/remove block with probability,
            //You should use setUnitBlock to control the occupancy probability.
            //the updateLogOddsOccupancy will be added on the log odds occupancy of block to in/decrease the occupancy
            //probabilistic set occupancy
            void setUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos, float updateLogOddsOccupancy);

            // check whether people can stand on this position or not, 
            // which means there is no obstacle or block here
            // and there is a block under it.
            // binary
            bool checkStandable(const BlockVector &pos) const;
            // probabilistic
            bool checkStandable(const BlockVector &pos, float logOddsOccupancy) const;
            // binary
            //Handle getBlock(const BlockVector& pos) const;
            // probabilistic
            //Handle getBlock(const BlockVector& pos, float logOddsOccupancy) const;
            // binary
            BlockVector getBlockLocation(const Handle& block) const;
            // probabilistic
            BlockVector getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const;

            /**
             *  public member functions for entity
             */
			
            // currently we consider the none block entity has no collision,
            // avatar can get through them
            void addNoneBlockEntity(const Handle& entityNode, 
                                    const BlockVector& pos,
                                    bool isSelfObject,
                                    bool isAvatarEntity,
                                    const unsigned long timestamp);
            void removeNoneBlockEntity(const Handle &entityNode);
            void updateNoneBlockEntityLocation(const Handle& entityNode, BlockVector newpos, 
                                               unsigned long timestamp);
            // note that we didn't delete the record 
            // when calling removeNoneBlockEntity()
            BlockVector getLastAppearedLocation(const Handle& entityHandle) const;
            Handle getEntity(const BlockVector& pos) const;

            /**
             * function for save/load map in persist/, but haven't implemented yet. Keep it to make code compiled.
             */
            void save(FILE* fp ){};
            void load(FILE* fp ){};
            static std::string toString( const OctomapOcTree& map ){return string("");}
            static OctomapOcTree* fromString( const std::string& map ){return NULL;}

        protected:

            AtomSpace*      mAtomSpace;
            std::string     mMapName;
            int             mFloorHeight; // the z of the floor
            float           mAgentHeight;
            int             mTotalUnitBlockNum;
            Handle          mSelfAgentEntity;
            // We keep the map for quick search position. 
            //Memory consuming: 50k blocks take about 10M RAM for one map
            map<Handle, BlockVector> mAllUnitAtomsToBlocksMap;
            set<Handle> mAllNoneBlockEntities;
            set<Handle> mAllAvatarList;
            multimap<BlockVector, Handle> mPosToNoneBlockEntityMap;
            map< Handle, vector< pair<unsigned long,BlockVector> > > mNoneBlockEntitieshistoryLocations;

            /**
             *    Inner helper function.
             */

            void _addNonBlockEntityHistoryLocation(Handle entityHandle,BlockVector newLocation, unsigned long timestamp);

            // this constructor is only used for clone
            OctomapOcTree(const OctomapOcTree&);
            /*
            OctomapOctree(string_MapName,
                          int _FloorHeight, int _AgentHeight,
                          int _TotalUnitBlockNum,Handle _mSelfAgentEntity,
                          AtomSpace* _AtomSpace,
                          const map<Handle, BlockVector>& _AllUnitAtomsToBlocksMap,
                          const set<Handle>& _AllNoneBlockEntities, 
                          const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
                          const set<Handle>& _AllAvatarList,
                          const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations);
            */

        };
        /************
              Working Now to move the interface of Octree3DMapManger!!
        *************/

    } // end namespace
}
#endif
