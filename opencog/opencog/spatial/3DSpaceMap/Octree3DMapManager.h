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

#include <opencog/atomspace/Handle.h>

#include "Block3DMapUtil.h"
#include "Block3D.h"
#include "Octree.h"
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
        class Entity3D;
        class BlockEntity;
        class Octree;
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
			Octree3DMapManager(const string& mapName,const unsigned& resolution, const int floorHeight);
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
            void removeSolidUnitBlock(const Handle& blockNode);
            bool checkIsSolid(double x, double y, double z);
            bool checkIsSolid(const BlockVector& pos);
            // check whether people can stand on this position or not, 
			// which means there is no obstacle or block here
			// and there is a block under it.
            bool checkStandable(const BlockVector &pos) const;
            bool checkStandable(double x, double y, double z) const;
            Block3D* getBlockAtLocation(double x, double y, double z);
			Block3D* getBlockAtLocation(const BlockVector& pos);
            Handle getUnitBlockHandleFromPosition(const BlockVector &pos);
            BlockVector getPositionFromUnitBlockHandle(const Handle &h);
            HandleSeq getAllUnitBlockHandlesOfABlock(Block3D& _block);

			/**
			 * public member functions about BlockEntity add/remove/query
			 */

            // If there is not a blockEntity here, return 0
            BlockEntity* getBlockEntityInPos(BlockVector& _pos) const;
            // this should be call only once 
			// just after perception finishes at the first time in embodiment.
            void findAllBlockEntitiesOnTheMap();
            // Given a posititon, find the BlockEntity the posititon belongs to
            BlockEntity* findAllBlocksInBlockEntity(BlockVector& _pos);
            BlockEntity* findBlockEntityByHandle(const Handle entityNode) const;
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

			/**
			 *  public member functions for entity
			 */

            // currently we consider the none block entity has no collision,
			// avatar can get through them
            void addNoneBlockEntity(const Handle &entityNode, 
									BlockVector _centerPosition,
                                    int _width, int _lenght, int _height, 
									double yaw, string _entityName,
									string _entityClass, bool isSelfObject,
									unsigned long timestamp,
									bool is_obstacle = false);

            void removeNoneBlockEntity(const Handle &entityNode);
            const Entity3D* getEntity(const Handle& entityNode) const;
            const Entity3D* getEntity(const string& entityName) const;
            bool isAvatarEntity(const Entity3D* entity) const;

            void updateNoneBLockEntityLocation(
				const Handle &entityNode, BlockVector _newpos, 
				unsigned long timestamp, bool is_standLocation = false);

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

            // get the last location this nonBlockEntity appeared
            BlockVector getLastAppearedLocation(Handle entityHandle);
            // to record all the history locations / centerPosition 
            // map <EntityHandle, vector< pair < timestamp, location> >
            map< Handle, vector< pair<unsigned long,BlockVector> > > nonBlockEntitieshistoryLocations;

			/**
			 *  member functions about Object add/remove/query
			 *  "Object" means the block,entity and blockentity
			 */

            BlockVector getObjectLocation(const Handle& objNode) const;
            BlockVector getObjectLocation(const string& objName) const;
            bool containsObject(const Handle& objectNode) const;
            bool containsObject(const string& objectname) const;

            // return the Direction of the given object face to.
            // we just define the direction of block is
			// BlockVector::X_UNIT direction (1,0,0).
            // if there is nothing for this handle on this map, 
			// return BlockVector::Zero
            BlockVector getObjectDirection(const Handle& objNode) const;

			// TODO: now we only count the entities, 
			// we may need to return all the unit blocks as well
			template<typename Out>
				Out getAllObjects(Out out) const
			{
                return findAllEntities(out);
			}

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
			 *  member functions about computation which doesn't use SpaceMap
			 */

            // get a random near position 
			// for building a same blockEnity as the _entity
            // this position should have enough space to build this new entity,
            // not to be overlapping other objects in the map.
            BlockVector getBuildEntityOffsetPos(BlockEntity* _entity) const;
            double distanceBetween(const Entity3D* entityA,
								   const Entity3D* entityB) const;
            double distanceBetween(const BlockVector& posA, 
								   const BlockVector& posB) const;
            double distanceBetween(const string& objectNameA, 
								   const string& objectNameB) const;
            double distanceBetween(const string& objectName, 
								   const BlockVector& pos) const;
            // Threshold to consider an entity next to another
            inline double getNextDistance() const { return AccessDistance;}
            bool isTwoPositionsAdjacent(const BlockVector& pos1, 
										const BlockVector& pos2);
            /**
             * Finds the list of spatial relationships 
			 * that apply to the three entities.
             * Currently this can only be BETWEEN, 
			 * which states that A is between B and C
             *
             * @param observer The observer entity
             * @param entityB First reference entity
             * @param entityC Second reference entity
             *
             * @return std::vector<SPATIAL_RELATION> 
			 *         a vector of all spatial relations
             *         among entityA (this entity), entityB (first reference) 
			 *         and entityC (second reference)
             *
             */
            std::set<SPATIAL_RELATION> computeSpatialRelations(
				const Entity3D* entityA,
				const Entity3D* entityB,
				const Entity3D* entityC = 0,
				const Entity3D* observer = 0) const;
            std::set<SPATIAL_RELATION> computeSpatialRelations(
				const string& entityAName,
				const string& entityBName,
				const string& entityCName = "",
				const string& observerName = "") const;
            std::set<SPATIAL_RELATION> computeSpatialRelations( 
				const AxisAlignedBox& boundingboxA,
				const AxisAlignedBox& boundingboxB,
				const AxisAlignedBox& boundingboxC = AxisAlignedBox::ZERO,
				const Entity3D* observer = 0 ) const;
            static string spatialRelationToString(SPATIAL_RELATION relation);


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
			  Dead interface

			  // Note: for non-super blockEntities only
			  // Find all the neighbour BlockEntities for every blockEntity 
			  //and describe their adjacent situation
			  void computeAllAdjacentBlockClusters();

			 */
        protected:

            std::string     mMapName;

            Octree*         mRootOctree;
			OctomapOcTree*  mOctomapOctree;

            int             mFloorHeight; // the z of the floor
            int             mAgentHeight;
            int             mTotalUnitBlockNum;
            static const int AccessDistance = 2;

            Entity3D* selfAgentEntity;

            // We keep these 2 map for quick search. 
			//Memory consuming: 50k blocks take about 10M RAM for one map
            map<Handle, BlockVector> mAllUnitAtomsToBlocksMap;
            map<BlockVector,Handle> mAllUnitBlocksToAtomsMap;

            map<int,BlockEntity*> mBlockEntityList;
            map<int,BlockEntity*> mSuperBlockEntityList;
            map<Handle, Entity3D*> mAllNoneBlockEntities;
            map<Handle, Entity3D*> mAllAvatarList;
            multimap<BlockVector, Entity3D*> mPosToNoneBlockEntityMap;

			/**
			 *    Inner helper function.
			 */

            // just remove this entity from the mBlockEntityList, but not delete it yet
            void removeAnEntityFromList(BlockEntity* entityToRemove);
			//for findEntities template
			string getEntityName(Entity3D* entity) const;

            bool getUnitBlockHandlesOfABlock(const BlockVector& _nearLeftPos, int _blockLevel, HandleSeq &handles);

            void _addNonBlockEntityHistoryLocation(Handle entityHandle,BlockVector newLocation, unsigned long timestamp);

            // this constructor is only used for clone
            Octree3DMapManager(bool _enable_BlockEntity_Segmentation, string  _MapName,OctomapOcTree* _OctomapOctree, int _FloorHeight, int _AgentHeight,
                               int _TotalUnitBlockNum,Entity3D* _selfAgentEntity,map<Handle, BlockVector>& _AllUnitAtomsToBlocksMap,
                               map<BlockVector,Handle>& _AllUnitBlocksToAtomsMap,map<int,BlockEntity*>& _BlockEntityList,map<Handle,
                               Entity3D*>& _AllNoneBlockEntities, map<Handle, vector<pair<unsigned long, BlockVector> > > _nonBlockEntitieshistoryLocations);


/*
#ifdef HAVE_ZMQ
            // using zmq to communicate with the learning server
            string fromLSIP;
            string fromLSPort;

            string toLSIP;
            string toLSPort;

            zmq::context_t * zmqLSContext;
            zmq::socket_t * socketSendToLS;
            zmq::socket_t * socketLSFromLS;
#endif // HAVE_ZMQ

            bool enableStaticsMapLearning;
 */

        };

    }
/** @}*/
}

#endif // _SPATIAL_NEW_OCTREE3DMAPMANAGER_H
