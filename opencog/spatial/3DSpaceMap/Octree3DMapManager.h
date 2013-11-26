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

#ifndef _SPATIAL_OCTREE3DMAPMANAGER_H
#define _SPATIAL_OCTREE3DMAPMANAGER_H

#include <map>
#include <set>
#include <vector>
#include <limits.h>

#ifdef HAVE_ZMQ
#include <zmq.hpp>
#endif

#include <opencog/atomspace/Handle.h>

#include "Block3DMapUtil.h"
#include "Block3D.h"
#include "Octree.h"

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

            // to store the blockEntities's node handles just diasppear,
            // the ~Blockentity() will add its Handel into this list, DO NOT add to this list from other place
            vector<Handle>  newDisappearBlockEntityList;

            // to store the blockEntities just appear
            // the Blockentity() will add itself into this list, DO NOT add to this list from other place
            vector<BlockEntity*> newAppearBlockEntityList;

            // to store the blockentities need to be updated the predicates
            vector<BlockEntity*> updateBlockEntityList;

            // to store the super blockEntities need to be updated the predicates
            vector<BlockEntity*> updateSuperBlockEntityList;

            static const int AccessDistance = 2;

            bool enable_BlockEntity_Segmentation;

            /**
             * @ min_x and min_y is the start position of this octree space
             * @_floorHeight: the height of the floor, the z value of  start position
             * @_offSet: how many unit per edge in this space, indicating the size of the whole space
             */
            Octree3DMapManager(std::string _mapName, int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight);
            ~Octree3DMapManager();

            //deep clone this octree3DMapManager and return the new instance
            // the cloned octree3DMapManager will have the same octress,blocks and entities,
            // and all these are new instances, shared the same properties and atom handles with the old ones
            Octree3DMapManager* clone();

            bool hasPerceptedMoreThanOneTimes;

            const map<Handle, BlockVector>& getAllUnitBlockatoms() const {return mAllUnitAtomsToBlocksMap;}

            const map<int,BlockEntity*>& getBlockEntityList() const {return mBlockEntityList;}

            const map<int,BlockEntity*>& getSuperBlockEntityList() const {return mSuperBlockEntityList;}

            const map<Handle, Entity3D*>& getAllNoneBlockEntities() const {return mAllNoneBlockEntities;}

            const map<Handle, Entity3D*>& getAllAvatarList() const {return mAllAvatarList;}

            int getTotalDepthOfOctree() const {return mTotalDepthOfOctree;}

            inline  int   getFloorHeight() const {return mFloorHeight;}
            inline  int   getTotalUnitBlockNum() const {return mTotalUnitBlockNum;}
            inline  const AxisAlignedBox& getMapBoundingBox() const {return mMapBoundingBox;}
            inline  std::string  getMapName()const {return mMapName;}

            inline  int   xMin() const{return mMapBoundingBox.nearLeftBottomConer.x;}
            inline  int   yMin() const{return mMapBoundingBox.nearLeftBottomConer.y;}
            inline  int   zMin() const{return mMapBoundingBox.nearLeftBottomConer.z;}

            inline  int   xMax() const{return mMapBoundingBox.nearLeftBottomConer.x + mMapBoundingBox.size_x;}
            inline  int   yMax() const{return mMapBoundingBox.nearLeftBottomConer.y + mMapBoundingBox.size_y;}
            inline  int   zMax() const{return mMapBoundingBox.nearLeftBottomConer.z + mMapBoundingBox.size_z;}

            inline  int   xDim() const{return mMapBoundingBox.size_x;}
            inline  int   yDim() const{return mMapBoundingBox.size_y;}
            inline  int   zDim() const{return mMapBoundingBox.size_z;}

            // currently we consider all the none block entities has no collision, agents can get through them
            void addNoneBlockEntity(const Handle &entityNode, BlockVector _centerPosition,
                                    int _width, int _lenght, int _height, double yaw, std::string _entityName,std::string _entityClass, bool isSelfObject,unsigned long timestamp,bool is_obstacle = false);

            void updateNoneBLockEntityLocation(const Handle &entityNode, BlockVector _newpos, unsigned long timestamp, bool is_standLocation = false);

            void removeNoneBlockEntity(const Handle &entityNode);

            void addSolidUnitBlock(BlockVector _pos, const Handle &_unitBlockAtom = opencog::Handle::UNDEFINED,  std::string _materialType = "", std::string _color = "" );

            // return the BlockEntity occupied this position, then the atomspace can update the predicates for this Entity
            // But if this entity is disappear during this process, then will return 0
            void removeSolidUnitBlock(const Handle &blockNode);

            // Given a posititon, find all the blocks in the BlockEntity this posititon belongs to.
            BlockEntity* findAllBlocksInBlockEntity(BlockVector& _pos);
            inline const Octree* getRootOctree(){return mRootOctree;}

            // get a random near position for building a same blockEnity as the _entity
            // this position should have enough space to build this new entity,
            // not to be overlapping other objects in the map.
            BlockVector getBuildEnityOffsetPos(BlockEntity* _entity) const;

            // Return the blockEntity occupies this postiton
            // If there is not a blockEntity here, return 0
            BlockEntity* getEntityInPos(BlockVector& _pos) const;

            // this should be call only once just after the map perception finishes at the first time in the embodiment.
            void findAllBlockEntitiesOnTheMap();

            // just remove this entity from the mBlockEntityList, but not delete it yet
            void removeAnEntityFromList(BlockEntity* entityToRemove);

            const Entity3D* getEntity( const Handle entityNode ) const;

            const Entity3D* getEntity( std::string entityName) const;

            std::string getEntityName(Entity3D* entity) const;

            // return the location of the given object handle. This object can be a block,nonblockentity or blockentity
            BlockVector getObjectLocation(Handle objNode) const;

            // return the location of the given object name. This object can be a block,nonblockentity or blockentity
            BlockVector getObjectLocation(std::string objName) const;

            // return the Direction of the given object face to. This object can be a block,nonblockentity or blockentity
            // Because it make no sense if this object is a block, we just define the directions of all the blocks are all BlockVector::X_UNIT direction (1,0,0)
            // if there is nothting for this handle on this map, return BlockVector::Zero
            BlockVector getObjectDirection(Handle objNode) const;

            BlockEntity* findBlockEntityByHandle(const Handle entityNode) const;

            /**
             * Find a free point near a given position, at a given distance
             * @param position Given position
             * @param distance Maximum distance from the given position to search the free point
             * @param startDirection Vector that points to the direction of the first rayTrace
             * @param toBeStandOn if this is true then agent can stand at that position,which means the point should not be on the sky
             * x
             */
            BlockVector getNearFreePointAtDistance( const BlockVector& position, int distance, const BlockVector& startDirection, bool toBeStandOn = true ) const;

            // check whether people can stand on this position or not, which means first there is not any obstacle or block here and there is a block under it.
            bool checkStandable(const BlockVector &pos) const;
            bool checkStandable(int x, int y, int z) const;

            bool containsObject(const Handle objectNode) const;
            bool containsObject(std::string& objectname) const;

            double distanceBetween(const Entity3D* entityA,const Entity3D* entityB) const;
            double distanceBetween(const BlockVector& posA, const BlockVector& posB) const;
            double distanceBetween(std::string objectNameA, std::string objectNameB) const;
            double distanceBetween(std::string objectName, const BlockVector& pos) const;

            static bool isTwoPositionsAdjacent(const BlockVector& pos1, const BlockVector& pos2);


            /**
             * TODO: Persistence
             */
            void save(FILE* fp ){};
            void load(FILE* fp ){};

            static std::string toString( const Octree3DMapManager& map );

            static Octree3DMapManager* fromString( const std::string& map );


            template<typename Out>
                 Out findAllEntities(Out out) const
            {

                 // only calculate the non-block entities and block entities, no including the blocks
                 std::vector<const char*> objectNameList;

                 // non-block entities:
                 map<Handle, Entity3D*> ::const_iterator it;

                 for ( it = mAllNoneBlockEntities.begin( ); it != mAllNoneBlockEntities.end( ); ++it )
                 {
                     objectNameList.push_back(getEntityName((Entity3D*)(it->second)).c_str( ));
                 } // for

                 return std::copy(objectNameList.begin(), objectNameList.end(), out);
             }


                 // todo: now we only count the entities, we may need to return all the unit blocks as well
             template<typename Out>
                     Out getAllObjects(Out out) const
             {
                return findAllEntities(out);
             }

            /*
             * Threshold to consider an entity next to another
             * @return Next distance
             */
            inline double getNextDistance( void ) const {
                return 2.0;
            }

            /**
             * Finds the list of spatial relationships that apply to the three entities.
             * Currently this can only be BETWEEN, which states that A is between B and C
             *
             * @param observer The observer entity
             * @param entityB First reference entity
             * @param entityC Second reference entity
             *
             * @return std::vector<SPATIAL_RELATION> a vector of all spatial relations
             *         among entityA (this entity), entityB (first reference) and entityC
             *         (second reference)
             *
             */
            std::set<SPATIAL_RELATION> computeSpatialRelations(
                                                                   const Entity3D* entityA,
                                                                   const Entity3D* entityB,
                                                                   const Entity3D* entityC = 0,
                                                                   const Entity3D* observer = 0) const;

            std::set<SPATIAL_RELATION> computeSpatialRelations(
                                                                   string entityAName,
                                                                   string entityBName,
                                                                   string entityCName = "",
                                                                   string observerName = ""
                                                                   ) const;

            std::set<SPATIAL_RELATION> computeSpatialRelations( const AxisAlignedBox& boundingboxA,
                                                                const AxisAlignedBox& boundingboxB,
                                                                const AxisAlignedBox& boundingboxC = AxisAlignedBox::ZERO,
                                                                const Entity3D* observer = 0 ) const;

            /**
             * Return a string description of the relation
             */
            static std::string spatialRelationToString( SPATIAL_RELATION relation );

            // Note: for non-super blockEntities only
            // Find all the neighbour BlockEntities for every blockEntity and describe their adjacent situation
            void computeAllAdjacentBlockClusters();

            int getAgentHeight(){return mAgentHeight;}
            void setAgentHeight(int _height){mAgentHeight = _height;}

            bool checkIsSolid(BlockVector& pos);
            bool checkIsSolid(int x, int y, int z);

            // return the handle of the unit block in this position
            Handle getUnitBlockHandleFromPosition(const BlockVector &pos);

            // return the position of this unit block given its handle
            BlockVector getPositionFromUnitBlockHandle(const Handle &h);

            HandleSeq getAllUnitBlockHandlesOfABlock(Block3D& _block);

            bool isAvatarEntity(const Entity3D* entity) const;

            // to recoard all the history locations/ centerPosition for all the nonBlockEntities, the lastest one is push_back
            // map <EntityHandle, vector< pair < timestamp, location> >
            map< Handle, vector< pair<unsigned long,BlockVector> > > nonBlockEntitieshistoryLocations;

            // get the last location this nonBlockEntity appeared
            BlockVector getLastAppearedLocation(Handle entityHandle);

        protected:

            // We keep these 2 map for quick search. Memory consuming: 50k blocks take about 10M RAM for one map
            map<Handle, BlockVector> mAllUnitAtomsToBlocksMap;
            map<BlockVector,Handle> mAllUnitBlocksToAtomsMap;

            map<int,BlockEntity*> mBlockEntityList;
            map<int,BlockEntity*> mSuperBlockEntityList;
            map<Handle, Entity3D*> mAllNoneBlockEntities;
            map<Handle, Entity3D*> mAllAvatarList;
            multimap<BlockVector, Entity3D*> mPosToNoneBlockEntityMap;

            int mTotalDepthOfOctree;

            std::string     mMapName;

            Octree*         mRootOctree;

            // Root octree has a depth of 1, everytime it splits, the depth ++
            // So till the deepest octree every block in it is a unit block

            int             mFloorHeight; // the z of the floor
            int             mAgentHeight;
            int             mTotalUnitBlockNum;

            // it's not the boundingbox for the map, not for the octree,
            // an octree boundingbox is usually a cube, but the map is not necessary to be a cube
            AxisAlignedBox mMapBoundingBox;

            Entity3D* selfAgentEntity;

            bool getUnitBlockHandlesOfABlock(const BlockVector& _nearLeftPos, int _blockLevel, HandleSeq &handles);

            void _addNonBlockEntityHistoryLocation(Handle entityHandle,BlockVector newLocation, unsigned long timestamp);

            // this constructor is only used for clone
            Octree3DMapManager(int _TotalDepthOfOctree,std::string  _MapName,Octree* _RootOctree, int _FloorHeight, int _AgentHeight,
                               int _TotalUnitBlockNum,AxisAlignedBox& _MapBoundingBox,Entity3D* _selfAgentEntity,map<Handle, BlockVector>& _AllUnitAtomsToBlocksMap,
                               map<BlockVector,Handle>& _AllUnitBlocksToAtomsMap,map<int,BlockEntity*>& _BlockEntityList,map<Handle,
                               Entity3D*>& _AllNoneBlockEntities, map<Handle, vector<pair<unsigned long, BlockVector> > > _nonBlockEntitieshistoryLocations, bool _enable_BlockEntity_Segmentation);


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

#endif // _SPATIAL_OCTREE3DMAPMANAGER_H
