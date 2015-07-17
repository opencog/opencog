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

    namespace spatial
    {
        class OctomapOcTree;
        enum SPATIAL_RELATION
        {
			LEFT_OF = 0,
            RIGHT_OF,
            ABOVE,
            BELOW,
            BEHIND,
            IN_FRONT_OF,
            BESIDE,
            NEAR,
            FAR_,
            TOUCHING, // touching is only face touching
            BETWEEN,
            INSIDE,
            OUTSIDE,

            TOTAL_RELATIONS
        };

        class Octree3DMapManager
        {
        public:
			Octree3DMapManager(AtomSpace& atomspace, const string& mapName,const unsigned& resolution, const int floorHeight);
            ~Octree3DMapManager();
            // deep clone this octree3DMapManager and return the new instance
            Octree3DMapManager* clone();

			/**
			 *   getter/setter
			 */
			
            inline int getFloorHeight() const {return mFloorHeight;}
            inline string getMapName() const {return mMapName;}
            inline int getAgentHeight() const {return mAgentHeight;}
            void setAgentHeight(int _height){mAgentHeight = _height;}
            inline unsigned getTotalDepthOfOctree() const {return mOctomapOctree->getTreeDepth();}
            inline int getTotalUnitBlockNum() const {return mTotalUnitBlockNum;}
			// Threshold to consider an entity next to another
			inline double getNextDistance() const { return AccessDistance;}
			BlockVector getKnownSpaceMinCoord() const;
			BlockVector getKnownSpaceMaxCoord() const;
			BlockVector getKnownSpaceDim() const;

			/**
			 *  public member functions about Block add/remove/query
			 */

			void addSolidUnitBlock(
				BlockVector _pos, 
				const Handle &_unitBlockAtom,
				string _materialType = "", string _color = "");
            void removeSolidUnitBlock(const Handle blockHandle);
            bool checkIsSolid(double x, double y, double z);
            bool checkIsSolid(const BlockVector& pos);
            // check whether people can stand on this position or not, 
			// which means there is no obstacle or block here
			// and there is a block under it.
            bool checkStandable(double x, double y, double z) const;
            bool checkStandable(const BlockVector &pos) const;
            Handle getBlockAtLocation(double x, double y, double z);
			Handle getBlockAtLocation(const BlockVector& pos);
			// For performance we especially save a map for BlockPosition and
			// block handle before; not sure if it's necessary after we replace
			// the old octree with octomap octree.
            Handle getUnitBlockHandleFromPosition(const BlockVector &pos);
            BlockVector getPositionFromUnitBlockHandle(const Handle &h);
			// Since we want to save info in atomspace it's not used.
            // HandleSeq getAllUnitBlockHandlesOfABlock(Handle& _block);

			/**
			 *  public member functions for entity
			 */
			
            // currently we consider the none block entity has no collision,
			// avatar can get through them
            void addNoneBlockEntity(const Handle& entityNode, 
									const BlockVector& pos,
                                    bool isSelfObject,
									const unsigned long timestamp);
            void removeNoneBlockEntity(const Handle &entityNode);
            //const Entity3D* getEntity(const Handle& entityNode) const;
            //const Entity3D* getEntity(const string& entityName) const;
            //bool isAvatarEntity(const Entity3D* entity) const;
            void updateNoneBLockEntityLocation(
				const Handle &entityNode, BlockVector newpos, 
			    unsigned long timestamp);
            // get the last location this nonBlockEntity appeared
            BlockVector getLastAppearedLocation(const Handle& entityHandle) const;
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
			 */

            BlockVector getObjectLocation(const Handle& objNode) const;
            BlockVector getObjectLocation(const string& objName) const;
            bool containsObject(const Handle& objectNode) const;
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
			 *  member functions about computation using OctomapOctree
			 */

            /**
             * Find a free point near a given position, at a given distance
             * @param position Given position
             * @param distance Maximum distance from the given position to search the free point
             * @param startDirection Vector that points to the direction of the first rayTrace
             * @param toBeStandOn if this is true then agent can stand at that position,which means the point should not be on the sky
             * x
             */
            BlockVector getNearFreePointAtDistance( 
				const BlockVector& position, int distance, 
				const BlockVector& startDirection, 
				bool toBeStandOn = true ) const;

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
            int             mAgentHeight;
            int             mTotalUnitBlockNum;
            Handle          selfAgentEntity;
            static const int AccessDistance = 2;


            // We keep these 2 map for quick search. 
			//Memory consuming: 50k blocks take about 10M RAM for one map
            map<Handle, BlockVector> mAllUnitAtomsToBlocksMap;
            map<BlockVector,Handle> mAllUnitBlocksToAtomsMap;
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
							   int _TotalUnitBlockNum,Handle _selfAgentEntity,
							   const map<Handle, BlockVector>& _AllUnitAtomsToBlocksMap,
                               const map<BlockVector,Handle>& _AllUnitBlocksToAtomsMap,
							   const set<Handle>& _AllNoneBlockEntities, 
							   const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
							   const set<Handle>& _AllAvatarList,
							   const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations);

			// not used since we save block info in atomspace
//     bool getUnitBlockHandlesOfABlock(const BlockVector& _nearLeftPos, int _blockLevel, HandleSeq &handles);
        };
    }
}

#endif // _SPATIAL_NEW_OCTREE3DMAPMANAGER_H

