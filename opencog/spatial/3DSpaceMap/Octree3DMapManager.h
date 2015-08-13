/*
 * opencog/spatial/3DSpaceMap/Octree3DMapManager.h
 *
 * Copyright (C) 2002-2011 OpenCog Foundation
 * All Rights Reserved
 * Author(s): Shujing Ke
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _SPATIAL_NEW_OCTREE3DMAPMANAGER_H
#define _SPATIAL_NEW_OCTREE3DMAPMANAGER_H


#include <map>
#include <set>
#include <vector>
#include <limits.h>

#ifdef HAVE_ZMQ
#include <lib/zmq/zmq.hpp>
#endif

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

#include "Block3DMapUtil.h"
#include "OctomapOctree.h"

using namespace std;

#define DOUBLE_MAX numeric_limits<double>::max()



namespace opencog
{
    /** \addtogroup grp_spatial
     *  @{
     */
	
    //Comment on 20150718 by YiShan
    //For now the 3DSpaceMap using Octomap as Octree to save block
    //So we provide the probabilistic feature for each function about block
    //You can directly use the interface without probability 
    //as if the occupancy is binary.
    //The library will control the occupancy probability automatically
    //But you can use the interface with probability
    //to control the occupancy of block.
    //You can also set the occupancy threshold to 
    //change the judgement of block occupancy.

    //Also, for the generic use of SpaceMap, we abandon the old
    //Block3D/Entity3D/BlockEntity class.
    //Since in different use case we want to save different infos.
    //It's better to save/query all the infos in AtomSpace.
    //And the SpaceMap should be used for indexing the block handle.
	
    //For now there are some parts unfinished
    //(1) add/remove/query BlockEntity
    //(2) spatial relation calculation
    //(We'll move the old function in MapManager to other place because they
    //are not related to Octree. Just a bunch of helper functions)
    //(3) add/remove/query nonUnitBlock(Maybe it's the same as BlockEntity..?)
	

    namespace spatial
    {
        class OctomapOcTree;
        class Octree3DMapManager
        {
        public:
            Octree3DMapManager(AtomSpace* atomspace, const string& mapName,const unsigned& resolution, const int floorHeight, const float agentHeight);
            ~Octree3DMapManager();
            // deep clone this octree3DMapManager and return the new instance
            Octree3DMapManager* clone();

            /**
             *   getter/setter
             */
			
            inline int getFloorHeight() const {return mFloorHeight;}
            inline string getMapName() const {return mMapName;}
            inline float getAgentHeight() const {return mAgentHeight;}
            void setAgentHeight(float _height){ mAgentHeight = _height;}
            inline unsigned getTotalDepthOfOctree() const {return mOctomapOctree->getTreeDepth();}
            inline int getTotalUnitBlockNum() const {return mTotalUnitBlockNum;}
            inline Handle getSelfAgentEntity() const {return mSelfAgentEntity;}
            // Note: logOdds(P)=log(P/(1-P))
            // in octomap api it usually express probabiblity by log odds
            inline float getLogOddsOccupiedThreshold() const {return mOctomapOctree->getOccupancyThresLog();}
            void setLogOddsOccupiedThreshold(float logOddsOccupancy);

            BlockVector getKnownSpaceMinCoord() const;
            BlockVector getKnownSpaceMaxCoord() const;
            BlockVector getKnownSpaceDim() const;

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
            Handle getBlock(const BlockVector& pos) const;
            // probabilistic
            Handle getBlock(const BlockVector& pos, float logOddsOccupancy) const;
            // binary
            BlockVector getBlockLocation(const Handle& block) const;
            // probabilistic
            BlockVector getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const;

            float getBlockLogOddsOccupancy(const BlockVector& pos) const;

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

            //The following is olde interface to get Entity3D
            //Since now we query the info in atomspace. It's not used.

            //const Entity3D* getEntity(const Handle& entityNode) const;
            //const Entity3D* getEntity(const string& entityName) const;
            //bool isAvatarEntity(const Entity3D* entity) const;
            /*
              template<typename Out>
              Out findAllEntities(Out out) const
              {
              std::vector<const char*> objectNameList;

              for ( auto it = mAllNoneBlockEntities.begin( ); 
              it != mAllNoneBlockEntities.end(); ++it )
              {
              objectNameList.push_back(getEntityName((Entity3D*)(it->second)).c_str());

              } 
              return std::copy(objectNameList.begin(), 
              objectNameList.end(), out);
              }
            */

            /**
             *  member functions about Object add/remove/query
             *  "Object" means the block,entity and blockentity
             *  Note that because we will query info in AtomSpace
             *  It should be easy to use atom type to know
             *  what the handle is (block/entity).
             *  so it's redundant to have these unclear "XXXObject" func.
             */

			
            //BlockVector getObjectLocation(const Handle& objNode) const;
            //BlockVector getObjectLocation(const string& objName) const;

            //bool containsObject(const string& objectname) const;
            // return the Direction of the given object face to.
            // we just define the direction of block is
            // BlockVector::X_UNIT direction (1,0,0).
            // if there is nothing for this handle on this map, 
            // return BlockVector::Zero
            //BlockVector getObjectDirection(const Handle& objNode) const;
            // TODO: now we only count the entities, 
            // we may need to return all the unit blocks as well
            /*
              template<typename Out>
              Out getAllObjects(Out out) const
              {
              return findAllEntities(out);
              }
            */

            /**
             * function for saving file; but not finished
             */
            void save(FILE* fp ){};
            void load(FILE* fp ){};
            static std::string toString( const Octree3DMapManager& map );
            static Octree3DMapManager* fromString( const std::string& map );

            /**
             *  parameter for old embodiment.
             */

            bool enable_BlockEntity_Segmentation;
            bool hasPerceptedMoreThanOneTimes;

            /*
            // Comment on 20150713 by Yi-Shan,
            // The following is old public functions about BlockEntity add/remove/query
            // Because the BlockEntity feature has not been designed well, 
            // so we comment out all the code related to BlockEntity
            // Once we need to use it/decide to do it, maybe we'll need the legacy code.

            // If there is not a blockEntity here, return 0
            BlockEntity* getBlockEntityInPos(BlockVector& _pos) const;
            // this should be call only once 
            // just after perception finishes at the first time in embodiment.
            void findAllBlockEntitiesOnTheMap();
            // Given a posititon, find the BlockEntity the posititon belongs to
            BlockEntity* findAllBlocksInBlockEntity(BlockVector& _pos);
            //Since we want to save info in atomspace it's not used.
            //BlockEntity* findBlockEntityByHandle(const Handle entityNode) const;
            // for building a same blockEnity as the _entity
            // this position should have enough space to build this new entity,
            // not to be overlapping other objects in the map.
            BlockVector getBuildEntityOffsetPos(BlockEntity* _entity) const;
            // just remove this entity from the mBlockEntityList, but not delete it yet
            void removeAnEntityFromList(BlockEntity* entityToRemove);
            // to store the blockEntities's node handles just diasppear,
            // the ~Blockentity() will add its Handle into this list, 
            // DO NOT add to this list from other place
            vector<Handle>  newDisappearBlockEntityList;
            // to store the blockEntities just appear
            // the Blockentity() will add itself into this list, 
            // DO NOT add to this list from other place
            vector<BlockEntity*> newAppearBlockEntityList;
            // to store the blockentities need to be updated the predicates
            vector<BlockEntity*> updateBlockEntityList;
            */


            /*
              Dead interface
              // Note: for non-super blockEntities only
              // Find all the neighbour BlockEntities for every blockEntity 
              //and describe their adjacent situation
              void computeAllAdjacentBlockClusters();
            */
        protected:

            AtomSpace*      mAtomSpace;
            std::string     mMapName;
            OctomapOcTree*  mOctomapOctree;
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
            // to record all the history locations / centerPosition 
            // map <EntityHandle, vector< pair < timestamp, location> >
            map< Handle, vector< pair<unsigned long,BlockVector> > > nonBlockEntitieshistoryLocations;

            // Comment on 20150713 by Yi-Shan,
            // Because the BlockEntity feature has not designed well, 
            // so we comment out all the code related to BlockEntity
            // Once we need to use it/decide to do it, maybe we'll need the legacy code.
            //map<Handle, BlockEntity*> mBlockEntityList;

            /**
             *    Inner helper function.
             */

            //for findEntities template
            //string getEntityName(Entity3D* entity) const;
            void _addNonBlockEntityHistoryLocation(Handle entityHandle,BlockVector newLocation, unsigned long timestamp);

            // this constructor is only used for clone
            Octree3DMapManager(bool _enable_BlockEntity_Segmentation, 
                               string  _MapName,OctomapOcTree* _OctomapOctree,
                               int _FloorHeight, int _AgentHeight,
                               int _TotalUnitBlockNum,Handle _mSelfAgentEntity,
                               AtomSpace* _AtomSpace,
                               const map<Handle, BlockVector>& _AllUnitAtomsToBlocksMap,
                               const set<Handle>& _AllNoneBlockEntities, 
                               const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
                               const set<Handle>& _AllAvatarList,
                               const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations);

        };
    }
}

#endif // _SPATIAL_NEW_OCTREE3DMAPMANAGER_H

