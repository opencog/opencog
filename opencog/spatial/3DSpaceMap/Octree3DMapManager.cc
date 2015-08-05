/*
 * opencog/spatial/3DSpaceMap/Octree3DMapManager.cc
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


#include <algorithm>
#include <iterator>
//#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>

// #ifdef HAVE_PROTOBUF
// #include <opencog/embodiment/Learning/LearningSpace/LearningMessage.info.pb.h>
// #endif
#include "Octree3DMapManager.h"
#include "SpaceMapUtil.h"
using namespace opencog;
using namespace opencog::spatial;

Octree3DMapManager::Octree3DMapManager(AtomSpace* atomspace, const std::string& mapName,const unsigned& resolution, const int floorHeight, const float agentHeight):
    mAtomSpace(atomspace), mMapName(mapName), mFloorHeight(floorHeight), mAgentHeight(agentHeight)
{	
    mOctomapOctree = new OctomapOcTree(resolution);
    mAllUnitAtomsToBlocksMap.clear();
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    nonBlockEntitieshistoryLocations.clear();
    hasPerceptedMoreThanOneTimes = false;

    selfAgentEntity = Handle::UNDEFINED;
    enable_BlockEntity_Segmentation = false;

    /*
    // Comment on 20150713 by Yi-Shan,
    // Because the BlockEntity feature has not designed well, 
    // so we comment out all the code related to BlockEntity
    // Once we need to use it/decide to do it, maybe we'll need the legacy code.
    mBlockEntityList.clear();    
    updateBlockEntityList.clear();
    newDisappearBlockEntityList.clear();
    newAppearBlockEntityList.clear();
    */

}

Octree3DMapManager::~Octree3DMapManager()
{
    // delete octree
    delete mOctomapOctree;
}

Octree3DMapManager* Octree3DMapManager::clone()
{
    Octree3DMapManager* cloneMap = new Octree3DMapManager(enable_BlockEntity_Segmentation,mMapName, mOctomapOctree,mFloorHeight,mAgentHeight,mTotalUnitBlockNum,selfAgentEntity, mAtomSpace, mAllUnitAtomsToBlocksMap, mAllNoneBlockEntities, mPosToNoneBlockEntityMap, mAllAvatarList, nonBlockEntitieshistoryLocations);
    return cloneMap;
}

void Octree3DMapManager::setLogOddsOccupiedThreshold(float logOddsOccupancy)
{
    mOctomapOctree->setOccupancyThres(logOddsOccupancy);
}

BlockVector Octree3DMapManager::getKnownSpaceMinCoord() const
{
    BlockVector coord;
    mOctomapOctree->getMetricMin(coord.x,coord.y,coord.z);
    return coord;
}
BlockVector Octree3DMapManager::getKnownSpaceMaxCoord() const
{
    BlockVector coord;
    mOctomapOctree->getMetricMax(coord.x,coord.y,coord.z);
    return coord;
}
BlockVector Octree3DMapManager::getKnownSpaceDim() const
{
    BlockVector dim;
    mOctomapOctree->getMetricSize(dim.x,dim.y,dim.z);
    return dim;
}


void Octree3DMapManager::addSolidUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos)
{
    float occupiedthres=mOctomapOctree->getOccupancyThresLog();
    setUnitBlock(_unitBlockAtom,_pos,occupiedthres);

    /*
    // Comment on 20150713 by Yi-Shan,
    // Because the BlockEntity feature has not designed well, 
    // so we comment out all the code related to BlockEntity
    // Once we need to use it/decide to do it, maybe we'll need the legacy code.
    if (this->enable_BlockEntity_Segmentation)
    {
    // first find all the entities will combined to this new block
    HandleSeq entities = mOctomapOctree->getNeighbourEntities(_pos);
    if (entities.size() == 0)
    {
    //there is not any entity will combine with this block
    // so make this block as a new entity which only contains one block
    Handle entity = addNewBlockEntity(atomSpace,Handleseq{blockHandle});
    mBlockEntityList.insert(entity);
    return;
    }
    else if (entities.size() == 1)
    {
    // only one entity combining, so just put this new block into that entity
    BlockEntity* entity = (BlockEntity*)(entities.front());
    entity->addBlock(block);
    updateBlockEntityList.push_back(entity);
    return;
    }
    else
    {
    // there are existing more than 1 BlockEntities will combine with this new block
    // this new block join two or more existing entities together,
    // we remove these old entities, and add all their blocks to this new big one
    BlockEntity* entity = new BlockEntity(this,*block);
    vector<BlockEntity*>::iterator iter;
    for (iter = entities.begin(); iter != entities.end(); ++iter)
    {
    // add all the blocks in this old entity to the new big one
    entity->addBlocks((vector<Block3D*>&)((BlockEntity*)(*iter))->getBlockList());
    BlockEntity* oldEntity = *iter;
    // delete this old entity
				
    removeAnEntityFromList(oldEntity);
    delete oldEntity;
    }
    mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
    return;
    }
    }
    */

}


void Octree3DMapManager::removeSolidUnitBlock(const Handle blockHandle)
{
    map<Handle, BlockVector>::iterator it;
    it = mAllUnitAtomsToBlocksMap.find(blockHandle);
    if (it == mAllUnitAtomsToBlocksMap.end())
    {
        logger().error("Octree3DMapManager::removeSolidUnitBlock: Cannot find this unit block in space map!/n");
    }

    BlockVector pos=it->second;
    addSolidUnitBlock(Handle::UNDEFINED,pos);
}

/*
// Comment on 20150713 by Yi-Shan,
// Because the BlockEntity feature has not designed well, 
// so we comment out all the code related to BlockEntity
// Once we need to use it/decide to do it, maybe we'll need the legacy code.
BlockEntity* myEntity = block->mBlockEntity;
if (myEntity == 0)
return;
if (this->enable_BlockEntity_Segmentation)
{
vector<Block3D*> blockList = myEntity->getBlockList();
// if this block is the only block in its entity, just remove this entity
if (blockList.size() == 0)
{
delete myEntity;
return;
}
// After removing this unit block, it may split the Entities into two or more Entities
// we get any of its neighbour unit block and find all the blocks combined together for this neighbor block,
// if the number of all the blocks found is less than the total number in this entity,
// then it suggests that this block has splitted this entity into two or more entities
// First get any a neighbour solid pos
Block3D* neighbourBlock = 0;
BlockVector neighbourPos = mOctomapOctree->getNeighbourSolidBlockVector(pos, neighbourBlock);
if (neighbourPos == BlockVector::ZERO || neighbourBlock == 0)
return;
// find from this neighbour BlockVector all the combining blocks
vector<Block3D*> neighbourblockList = mOctomapOctree->findAllBlocksCombinedWith(&neighbourPos);
neighbourblockList.push_back(neighbourBlock);
if (neighbourblockList.size() == blockList.size())
{
// the block number is the same, so this removed block has not splitted this entity into two or more pieces
updateBlockEntityList.push_back(myEntity);
return;
}
else
{
// create new BlockEntities for the splitted parts of the original entity
// First clear all the blocks in the original entity
myEntity->clearAllBlocks();
// create the first entity we just found
BlockEntity* entity = new BlockEntity(this,neighbourblockList);
mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
// get all the neighbour solid positions of this removed unit block
vector<BlockVector> allNeighbours = mOctomapOctree->getAllNeighbourSolidBlockVectors(pos);
vector<BlockVector>::iterator it = allNeighbours.begin();
Block3D* curBlock;
for (; it != allNeighbours.end(); ++it)
{
// get the block in this position
if (! mOctomapOctree->checkIsSolid((BlockVector&)(*it), curBlock))
continue;
if (curBlock->mBlockEntity != 0)
continue;
// calculate from this block to find all the blocks combine with it
vector<Block3D*> newBlockList = mOctomapOctree->findAllBlocksCombinedWith(&*it);
// create a new entity for this part of original entity
BlockEntity* newEntity = new BlockEntity(this,newBlockList);
newEntity->addBlocks(newBlockList);
mBlockEntityList.insert(map<int,BlockEntity*>::value_type(newEntity->getEntityID(), newEntity));
}
// remove the original entity
removeAnEntityFromList(myEntity);
delete myEntity;
return;
}
}
*/

void Octree3DMapManager::setUnitBlock(const Handle& _unitBlockAtom, BlockVector _pos, float updateLogOddsOccupancy)
{
    if (mOctomapOctree->checkIsOutOfRange(_pos))
    {
        logger().error("addSolidUnitBlock: You want to add a unit block which outside the limit of the map: at x = %f, y = %f, z= %f ! /n",
                       _pos.x,_pos.y,_pos.z);
        return;
    }
    Handle oldBlock=mOctomapOctree->getBlock(_pos);

    if(oldBlock==Handle::UNDEFINED && _unitBlockAtom!=Handle::UNDEFINED)
    { 
        mTotalUnitBlockNum++;
        mAllUnitAtomsToBlocksMap.insert(pair<Handle, BlockVector>(_unitBlockAtom, _pos));

    }
    else if(oldBlock!=Handle::UNDEFINED && _unitBlockAtom==Handle::UNDEFINED)
    { 
        mTotalUnitBlockNum--;
        mAllUnitAtomsToBlocksMap.erase(oldBlock);		
    }
    mOctomapOctree->setBlock(_unitBlockAtom, _pos, updateLogOddsOccupancy);
}

bool Octree3DMapManager::checkIsSolid(const BlockVector& pos) const
{
    Handle blockHandle=mOctomapOctree->getBlock(pos);
    return (blockHandle!=Handle::UNDEFINED);
}

bool Octree3DMapManager::checkIsSolid(const BlockVector& pos, float logOddsOccupancy) const
{
    Handle blockHandle=mOctomapOctree->getBlock(pos,logOddsOccupancy);
    return (blockHandle!=Handle::UNDEFINED);
}

bool Octree3DMapManager::checkStandable(const BlockVector& pos) const
{
    return checkStandable(pos,mOctomapOctree->getOccupancyThresLog());
}

bool Octree3DMapManager::checkStandable(const BlockVector &pos, float logOddsOccupancy) const
{
    if (mOctomapOctree->checkIsOutOfRange(pos))
    {
        logger().error("checkstandable: You want to add a unit block which outside the limit of the map: at x = %d, y = %d, z= %d ! /n",
                       pos.x,pos.y,pos.z);
        return false;
    }


    // check if there is any non-block obstacle in this pos
    Handle blockHandle=mOctomapOctree->getBlock(pos,logOddsOccupancy);
    if (blockHandle!=Handle::UNDEFINED)
        return false;

    if (pos.z <= mFloorHeight)
        return false;

    // because the agent has a height,
    // we should check if there are enough room above this pos for this agent to stand on
    if (mAgentHeight > 1)
    {
        for (int height = 1; height < mAgentHeight; height ++)
        {
            logger().error("count %f %f %f",pos.x,pos.y,pos.z+height);
            BlockVector blockAbove(pos.x,pos.y,pos.z + height);
            if (mOctomapOctree->getBlock(blockAbove,logOddsOccupancy)!=Handle::UNDEFINED)
            { 
                logger().error("in %f %f %f is not undef",pos.x,pos.y,pos.z+height);
                return false;
            }
        }
    }

    // because there are too many blocks for the whole floor, we don't send these floor blocks to opencog
    // so we just check if the pos height is just on the floor.
    if ( (pos.z - mFloorHeight) == 1)
        return true;

    BlockVector under(pos.x,pos.y,pos.z - 1);
    Handle underBlock=mOctomapOctree->getBlock(under,logOddsOccupancy);
    if (underBlock!=Handle::UNDEFINED)
    {
        //TODO:Judge if this block is standable
        //if agent can't stand on it (ex.water/lava)return false
        HandleSeq blocks;
        blocks.push_back(underBlock);
        string materialOfUnderBlock = (getPredicate(*mAtomSpace,"material",blocks,1))[0];
        if ( materialOfUnderBlock == "water") { 
            return false;
        } else {
            return true;
        }

			
    }	

    return false;
	
}


Handle Octree3DMapManager::getBlock(const BlockVector& pos) const
{
    return mOctomapOctree->getBlock(pos);
}

Handle Octree3DMapManager::getBlock(const BlockVector& pos, float logOddsOccupancy) const
{
    return mOctomapOctree->getBlock(pos,logOddsOccupancy);
}

BlockVector Octree3DMapManager::getBlockLocation(const Handle& block) const
{
    return getBlockLocation(block,mOctomapOctree->getOccupancyThresLog());
}
BlockVector Octree3DMapManager::getBlockLocation(const Handle& block, float logOddsOccupancyThreshold) const
{

    auto it = mAllUnitAtomsToBlocksMap.find(block);
    if (it == mAllUnitAtomsToBlocksMap.end())
    {return BlockVector::ZERO;}
    else
    {
        BlockVector result=it->second;		
        if(getBlockLogOddsOccupancy(result)<logOddsOccupancyThreshold){ return BlockVector::ZERO;}
        else { return result;} 
    }

}

float Octree3DMapManager::getBlockLogOddsOccupancy(const BlockVector& pos) const
{
    return mOctomapOctree->search(pos.x,pos.y,pos.z)->getLogOdds();
}

// currently we consider all the none block entities has no collision, agents can get through them
void Octree3DMapManager::addNoneBlockEntity(const Handle &entityNode, const BlockVector& pos,bool isSelfObject, bool isAvatarEntity, const unsigned long timestamp)
{
    auto it= mAllNoneBlockEntities.find(entityNode);    
    if (it == mAllNoneBlockEntities.end())
    {
        mAllNoneBlockEntities.insert(entityNode);
        mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(pos,entityNode));
        if (isSelfObject){ selfAgentEntity = entityNode;}
        if (isAvatarEntity){ mAllAvatarList.insert(entityNode);}
        _addNonBlockEntityHistoryLocation(entityNode,pos, timestamp);
    }
    else{ updateNoneBlockEntityLocation(entityNode,pos,timestamp);}
}

void Octree3DMapManager::_addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp)
{
    auto it = nonBlockEntitieshistoryLocations.find(entityHandle);
    if (it == nonBlockEntitieshistoryLocations.end())
    {
        vector< pair< unsigned long,BlockVector> > newVector;
        newVector.push_back(pair<unsigned long,BlockVector>(timestamp,newLocation));
        nonBlockEntitieshistoryLocations.insert(map< Handle, vector< pair<unsigned long,BlockVector> > >::value_type(entityHandle,newVector));
    }
    else
    {
        vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if (newLocation == (oneEntityHistories.back()).second)
        { return;} 
        else
        { 
            oneEntityHistories.push_back(pair<unsigned long,BlockVector>(timestamp,newLocation));    
        }
    }
}
// currently we consider all the none block entities has no collision, agents can get through them

void Octree3DMapManager::removeNoneBlockEntity(const Handle &entityNode)
{
    auto it = mAllNoneBlockEntities.find(entityNode);
    if (it != mAllNoneBlockEntities.end())
    {
        // delete from mPosToNoneBlockEntityMap first
        BlockVector pos = getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(pos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(pos);
        while(biter != eiter)
        {
            if (biter->second == entityNode)
            {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
        mAllNoneBlockEntities.erase(it);
    }
}

void Octree3DMapManager::updateNoneBlockEntityLocation(const Handle &entityNode, BlockVector newpos, unsigned long timestamp)
{

    mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(newpos,entityNode));

    if (mAllNoneBlockEntities.find(entityNode) != mAllNoneBlockEntities.end())
    {
        BlockVector lastpos=getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(lastpos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(lastpos);
        while(biter != eiter)
        {
            if (biter->second == entityNode)
            {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
    }
    _addNonBlockEntityHistoryLocation(entityNode,newpos, timestamp);
}

BlockVector Octree3DMapManager::getLastAppearedLocation(const Handle& entityHandle) const
{
    auto it = nonBlockEntitieshistoryLocations.find(entityHandle);
    if (it == nonBlockEntitieshistoryLocations.end())
        return BlockVector::ZERO;
    else
    {
        const vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if (oneEntityHistories.size() == 0)
            return BlockVector::ZERO;
        else
            return (oneEntityHistories.back()).second;
    }
}

Handle Octree3DMapManager::getEntity(const BlockVector& pos) const
{
    auto it=mPosToNoneBlockEntityMap.find(pos);
    if(it==mPosToNoneBlockEntityMap.end()){ return Handle::UNDEFINED;}
    else { return it->second;}
}

// this constructor is only used for clone

Octree3DMapManager::Octree3DMapManager(
                                       bool _enable_BlockEntity_Segmentation,
                                       string _MapName, OctomapOcTree *_OctomapOctree, int _FloorHeight,
                                       int _AgentHeight,int _TotalUnitBlockNum,Handle _selfAgentEntity,
                                       AtomSpace* _AtomSpace,
                                       const map<Handle, BlockVector> &_AllUnitAtomsToBlocksMap,
                                       const set<Handle> &_AllNoneBlockEntities,
                                       const multimap<BlockVector, Handle>& _PosToNoneBlockEntityMap,
                                       const set<Handle>& _AllAvatarList,
                                       const map<Handle, vector<pair<unsigned long, BlockVector> > >& _nonBlockEntitieshistoryLocations):
enable_BlockEntity_Segmentation(_enable_BlockEntity_Segmentation), mMapName(_MapName),mFloorHeight(_FloorHeight),
    mAgentHeight(_AgentHeight),mTotalUnitBlockNum(_TotalUnitBlockNum), selfAgentEntity(_selfAgentEntity)
{
    // the clone order should not be change here:
    // should always clone the octree before the entity list
    mOctomapOctree = new OctomapOcTree(*_OctomapOctree);
    // copy all unit blocks
    mAllUnitAtomsToBlocksMap = _AllUnitAtomsToBlocksMap;
    mAtomSpace=_AtomSpace;
    // copy all the NoneBlockEntities and mPosToNoneBlockEntityMap and mAllAvatarList
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    mAllAvatarList.clear();
    mAllNoneBlockEntities=_AllNoneBlockEntities;
    mPosToNoneBlockEntityMap=_PosToNoneBlockEntityMap;
    mAllAvatarList=_AllAvatarList;
    nonBlockEntitieshistoryLocations = _nonBlockEntitieshistoryLocations;
}


/**
 *    The following are unfinished/dead codes.
 *    (1)to/from String
 *    (2)BlockEntity feature
 */


// TOTO: to be finished
std::string Octree3DMapManager::toString( const Octree3DMapManager& map )
{

    std::stringstream out;
    out.precision(25);
    /*
      out << map.xMin( ) << " " << map.xMax( ) << " " << map.xDim( ) << " ";
      out << map.yMin( ) << " " << map.yMax( ) << " " << map.yDim( ) << " ";
      out << map.getFloorHeight() << " ";
      out << map.entities.size() << " ";
      LongEntityPtrHashMap::const_iterator it;
      for ( it = map.entities.begin( ); it != map.entities.end( ); ++it ) {
      out << it->second->getName( ) << " ";
      out << it->second->getPosition( ).x << " "
      << it->second->getPosition( ).y << " "
      << it->second->getPosition( ).z << " ";
      out << it->second->getLength( ) << " "
      << it->second->getWidth( ) << " "
      << it->second->getHeight( ) << " ";
      out << it->second->getOrientation( ).x << " "
      << it->second->getOrientation( ).y << " "
      << it->second->getOrientation( ).z << " "
      << it->second->getOrientation( ).w << " ";
      out << it->second->getStringProperty( Entity::ENTITY_CLASS ) << " ";
      out << it->second->getBooleanProperty( Entity::OBSTACLE ) << " ";
      } // for
    */
    return out.str( );
}

// TODO:

Octree3DMapManager* Octree3DMapManager::fromString( const std::string& map )
{	
    return NULL;

    /*
      std::stringstream parser(map);
      parser.precision(25);
      double xMin = 0, xMax = 0;
      double yMin = 0, yMax = 0;
      double floorHeight = 0;
      double agentRadius = 0;
      double agentHeight = 0;
      unsigned int xDim = 0, yDim = 0, numberOfObjects = 0;
      parser >> xMin >> xMax >> xDim;
      parser >> yMin >> yMax >> yDim;
      parser >> floorHeight;
      parser >> agentRadius;
      parser >> agentHeight;
      Octree3DMapManager* newMap = new Octree3DMapManager( xMin, xMax, xDim, yMin, yMax, yDim, floorHeight, agentRadius, agentHeight );
      parser >> numberOfObjects;
      unsigned int i;
      for( i = 0; i < numberOfObjects; ++i ) {
      std::string name;
      math::Vector3 position( 0, 0, 0);
      math::Dimension3 dimension;
      double orientationX = 0, orientationY = 0, orientationZ = 0, orientationW = 0;
      std::string entityClass;
      bool obstacle = false;
      parser >> name
      >> position.x >> position.y >> position.z
      >> dimension.length >> dimension.width >> dimension.height
      >> orientationX >> orientationY >> orientationZ >> orientationW
      >> entityClass
      >> obstacle;
      math::Quaternion orientation( orientationX, orientationY, orientationZ, orientationW );
      spatial::ObjectMetaData metaData( position.x,
      position.y,
      position.z,
      dimension.length,
      dimension.width,
      dimension.height,
      orientation.getRoll( ),
      entityClass);
      newMap->addObject( name, metaData, obstacle );
      } // for
      return newMap;
    */
}


//The following is unused functions about entity

/*
  const Entity3D* Octree3DMapManager::getEntity(const Handle& entityNode ) const
  {
  // first find in nonblockEnities
  map<Handle, Entity3D*> ::const_iterator it = mAllNoneBlockEntities.find(entityNode);
  if ( it != mAllNoneBlockEntities.end( ) )
  {
  return it->second;
  }
  }
*/
/*
	    
// Comment on 20150713 by Yi-Shan,
// Because the BlockEntity feature has not designed well, 
// so we comment out all the code related to BlockEntity
// Once we need to use it/decide to do it, maybe we'll need the legacy code.
else
{
// then find in blockEntities:
Entity3D* entity = findBlockEntityByHandle(entityNode);
if (entity)
return entity;
}
	
return 0;

}
*/
/*
  const Entity3D* Octree3DMapManager::getEntity(const string& entityName) const
  {
  // first find in nonblockEnities

  for (auto it = mAllNoneBlockEntities.begin( ); it != mAllNoneBlockEntities.end( ); ++it )
  {
  Entity3D* e = (Entity3D*)(it->second);
  if (e->getEntityName() == entityName)
  return e;
  }
   
  for (auto it2 = mBlockEntityList.begin( ); it2 != mBlockEntityList.end( ); ++it2 )
  {
  if (((BlockEntity*)(it2->second))->getEntityName() == entityName)
  return (it2->second);
  }

  return 0;
  }

  bool Octree3DMapManager::isAvatarEntity(const Entity3D *entity) const
  {
  string _entityClass = entity->getEntityClass() ;

  return( (_entityClass == "avatar") ||
  (_entityClass == "npc")    ||
  (_entityClass == "Player") ||
  (_entityClass == "player"));

  }
*/

/*
  bool Octree3DMapManager::containsObject(const string& objectName) const
  {
  const Entity3D* entity = getEntity(objectName);

  if (entity)
  return true;
  else
  return false;
  }
*/

/*
  bool Octree3DMapManager::containsObject(const Handle& objectNode) const
  {
  // there are 3 kinds of object on the map: nonbLockEnitities, blocks, and blockEntities,
  // look up in these 3 collection for this object
  if( mAllNoneBlockEntities.find(objectNode) == mAllNoneBlockEntities.end()
  && mAllUnitAtomsToBlocksMap.find(objectNode)== mAllUnitAtomsToBlocksMap.end())
  { return false;}
  else { return true;}
  }



  BlockVector Octree3DMapManager::getObjectLocation(const Handle& objNode) const
  {
  // first if this object is a block, return the position of this block
  map<Handle, BlockVector>::const_iterator it2;
  it2 = mAllUnitAtomsToBlocksMap.find(objNode);
  if (it2 != mAllUnitAtomsToBlocksMap.end())
  { return (BlockVector)(it2->second);}

  // if this object handle is not a block, then it should be an entity
  // if the handle is not found, it will return BlockVector::ZERO
  else { return getLastAppearedLocation(objNode);}

  }
*/


/*
  BlockVector Octree3DMapManager::getObjectLocation(const string& objName) const
  {
  const Entity3D* entity = getEntity(objName);
  if (entity == 0)
  return BlockVector::ZERO;

  return entity->getPosition();

  }
*/

/*
  BlockVector Octree3DMapManager::getObjectDirection(const Handle& objNode) const
  {
  // if it's a block, the direction make no sense, so we just use the x direction
  map<Handle, BlockVector>::const_iterator it2;
  it2 = mAllUnitAtomsToBlocksMap.find(objNode);
  if (it2 != mAllUnitAtomsToBlocksMap.end())
  return BlockVector::X_UNIT;

  Entity3D* entity = (Entity3D*)(getEntity(objNode));
  if (entity)
  return entity->getDirection();
  else
  return BlockVector::ZERO;
  }
*/
/*
  double Octree3DMapManager::distanceBetween(const Entity3D* entityA,const Entity3D* entityB) const
  {
  return (entityA->getPosition() - entityB->getPosition());
  }

  double Octree3DMapManager::distanceBetween(const BlockVector& posA, const BlockVector& posB) const
  {
  return (posA - posB);
  }

  double Octree3DMapManager::distanceBetween(const string& objectNameA,const string& objectNameB) const
  {
  const Entity3D* entityA = getEntity(objectNameA);
  const Entity3D* entityB = getEntity(objectNameB);
  if (entityA && entityB)
  return distanceBetween(entityA,entityB);
  else
  return DOUBLE_MAX;
  }

  double Octree3DMapManager::distanceBetween(const string& objectName, const BlockVector& pos) const
  {
  const Entity3D* entity = getEntity(objectName);
  if (! entity)
  return DOUBLE_MAX;

  return (entity->getPosition() - pos);

  }

  bool Octree3DMapManager::isTwoPositionsAdjacent(const BlockVector &pos1, const BlockVector &pos2)
  {
  int d_x = pos1.x - pos2.x;
  int d_y = pos1.y - pos2.y;
  int d_z = pos1.z - pos2.z;
  if (( d_x >=-1) && (d_x <= 1) &&
  ( d_y >=-1) && (d_y <= 1) &&
  ( d_z >=-1) && (d_z <= 1))
  {
  if ((d_x == 0) && (d_y == 0))
  return false; // the position just above or under is considered not accessable
  else
  return true;
  }

  return false;
  }

  // todo: to be completed

  std::set<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const AxisAlignedBox& boundingboxA,
  const AxisAlignedBox& boundingboxB,
  const AxisAlignedBox& boundingboxC,
  const Entity3D* observer ) const
  {
  std::set<SPATIAL_RELATION> spatialRelations;

  if (boundingboxC != AxisAlignedBox::ZERO)
  {
  // todo: compute if A is between B and C

  return spatialRelations;
  }

  if (observer == 0 )
  observer = selfAgentEntity;

  if (boundingboxA.isFaceTouching(boundingboxB))
  {
  spatialRelations.insert(TOUCHING);
  }

  if (boundingboxA.nearLeftBottomConer.z >= boundingboxB.nearLeftBottomConer.z + boundingboxB.size_z)
  spatialRelations.insert(ABOVE);

  if (boundingboxB.nearLeftBottomConer.z >= boundingboxA.nearLeftBottomConer.z + boundingboxA.size_z)
  spatialRelations.insert(BELOW);


  // if A is near/far to B
  double dis = boundingboxB.getCenterPoint() - boundingboxA.getCenterPoint();
  double AR = boundingboxA.getRadius();
  double BR = boundingboxB.getRadius();
  double nearDis = (AR + BR)*2.0;
  if (dis <= nearDis )
  spatialRelations.insert(NEAR);
  else if (dis > nearDis*10.0 )
  spatialRelations.insert(FAR_);

  return spatialRelations;
  }


  std::set<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const Entity3D* entityA,
  const Entity3D* entityB,
  const Entity3D* entityC,
  const Entity3D* observer ) const
  {
  if (entityC)
  {
  return computeSpatialRelations(entityA->getBoundingBox(),entityB->getBoundingBox(),entityC->getBoundingBox(),observer);
  }
  else
  {
  return computeSpatialRelations(entityA->getBoundingBox(),entityB->getBoundingBox(),AxisAlignedBox::ZERO,observer);
  }
  }


  std::string Octree3DMapManager::spatialRelationToString( SPATIAL_RELATION relation ) 
  {
  switch( relation ) {
  case LEFT_OF: return "left_of";
  case RIGHT_OF: return "right_of";
  case ABOVE: return "above";
  case BELOW: return "below";
  case BEHIND: return "behind";
  case IN_FRONT_OF: return "in_front_of";
  case BESIDE: return "beside";
  case NEAR: return "near";
  case FAR_: return "far";
  case TOUCHING: return "touching";
  case BETWEEN: return "between";
  case INSIDE: return "inside";
  case OUTSIDE: return "outside";
  default:
  case TOTAL_RELATIONS:
  return " invalid relation ";
  }
  }
*/




/*
//find the nearest entity to a given point satisfying some predicate
template<typename Pred>
ObjectID findNearestFiltered(const Point& p, Pred pred) const {
std::vector<ObjectID> tmp;
Distance d = 5;
//convert to greidPoint
GridPoint g = snap(p);
Distance x = fmax(xDim() - g.first - 1, g.first);
Distance y = fmax(yDim() - g.second - 1, g.second);
Distance maxD = sqrt((x * x) + (y * y));
ObjectIDSet objs;
struct rec_find finder(g, _grid, objs, *this);
struct rec_find finderNonObstacle(g, _grid_nonObstacle, objs, *this);
do {
finder.johnnie_walker(d);
finderNonObstacle.johnnie_walker(d);
std::copy(objs.begin(), objs.end(), back_inserter(tmp));
tmp.erase(std::partition(tmp.begin(), tmp.end(), pred), tmp.end());
if (d >= maxD && tmp.empty()) {
opencog::logger().debug("LocalSpaceMap - Found no object that verifies the predicate. Distance %.2f, Max distance %.2f.", d, maxD);
//won't find anything
return ObjectID();
}
d *= 1.5; //maybe it shouldn't grow so fast?
if (d > maxD ) {
d = maxD;
}
} while (tmp.empty());
//now find the closest
std::vector<Distance> dists(tmp.size());
std::transform(tmp.begin(), tmp.end(), dists.begin(),
boost::bind(&LocalSpaceMap2D::minDist, this,  ::_1, boost::cref(p)));
return *(tmp.begin() + distance(dists.begin(),
min_element(dists.begin(), dists.end())));
}
*/



/*
  HandleSeq Octree3DMapManager::getAllUnitBlockHandlesOfABlock(Handle& _block)
  {
  HandleSeq empty;
  HandleSeq handles;
  if (! getUnitBlockHandlesOfABlock(_block.getPosition(),_block.getLevel(),handles))
  return empty;
  else
  return handles;
  }
*/
/* a recursive function
   bool Octree3DMapManager::getUnitBlockHandlesOfABlock(const BlockVector& _nearLeftPos, int _blockLevel, HandleSeq &handles)
   {
   if (_blockLevel < 1)
   return false;
   if (_blockLevel == 1)
   {
   Handle h = getUnitBlockHandleFromPosition(_nearLeftPos);
   if (h == Handle::UNDEFINED)
   return false;
   handles.push_back(h);
   }
   else
   {
   int newLevel = _blockLevel - 1;
   for (int i = 0; i < 2; i ++)
   for (int j = 0; j < 2; j ++)
   for (int k = 0; k < 2; k ++)
   {
   BlockVector pos(_nearLeftPos.x + newLevel * i,_nearLeftPos.y + newLevel * j, _nearLeftPos.z + newLevel * k);
   return (getUnitBlockHandlesOfABlock(pos, newLevel, handles));
   TODO: TNick: my guess is that this was intended to be:
   //if ( !getUnitBlockHandlesOfABlock(pos, newLevel, handles)) )
   //    return false;
					 
   }
   }
   return true;
   }
*/


/*
// Comment on 20150713 by Yi-Shan,
// Because the BlockEntity feature has not designed well, 
// so we comment out all the code related to BlockEntity
// Once we need to use it/decide to do it, maybe we'll need the legacy code.
// Return the blockEntity occupies this postiton
// If there is not a blockEntity here, return 0
BlockEntity* Octree3DMapManager::getBlockEntityInPos(BlockVector& _pos) const
{
Block3D* block;
// First, check is there already a block in this position
if (! mOctomapOctree->checkIsSolid(_pos, block))

return 0;
return (block->mBlockEntity);
}
void Octree3DMapManager::findAllBlockEntitiesOnTheMap()
{
if (! enable_BlockEntity_Segmentation)
return;
map<Handle, BlockVector>::iterator it;
BlockVector pos;
Block3D* block;
int newEntitiesNum = 0;
for (it =mAllUnitAtomsToBlocksMap.begin(); it != mAllUnitAtomsToBlocksMap.end(); ++ it)
{
pos = (BlockVector)(it->second);
// First, check is there a block in this position
if (! mOctomapOctree->checkIsSolid(pos, block))
return;
// check this block has been already found in an existing Entity
if (block == 0)
return;
if (block->mBlockEntity == 0)
{
BlockEntity* entity = new BlockEntity(this,*block);
// find all the blocks combine with this block
vector<Block3D*> blockList = mOctomapOctree->findAllBlocksCombinedWith(&pos);
//  entity->SortBlockOrder();
entity->addBlocks(blockList);
mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
newEntitiesNum ++;
}
}
printf("Found %d BlockEntities in total! \n", newEntitiesNum);
}
BlockEntity* Octree3DMapManager::findAllBlocksInBlockEntity(BlockVector& _pos)
{
Block3D* block;
// First, check is there a block in this position
if (! mOctomapOctree->checkIsSolid(_pos, block))
return 0;
if (block->mBlockEntity != 0)
{
// We have ever calculate this BlockEntity, we calculate again
block->mBlockEntity->clearAllBlocks();
vector<Block3D*> blockList = mOctomapOctree->findAllBlocksCombinedWith(&_pos);
blockList.push_back(block);
block->mBlockEntity->addBlocks(blockList);
return block->mBlockEntity;
}
else
{
BlockEntity* entity = new BlockEntity(this,*block);
// find all the blocks combine with this block
vector<Block3D*> blockList = mOctomapOctree->findAllBlocksCombinedWith(&_pos);
entity->addBlocks(blockList);
mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
return entity;
}
}
BlockEntity* Octree3DMapManager::findBlockEntityByHandle(const Handle entityNode) const
{
for (auto iter = mBlockEntityList.begin(); 
iter != mBlockEntityList.end(); ++iter)
{
if (((BlockEntity*)(iter->second))->mEntityNode == entityNode)
{
return (BlockEntity*)(iter->second);
}
}
return NULL;
}
*/


/*Dead code
  void Octree3DMapManager::computeAllAdjacentBlockClusters()
  {
  TODO
  map<int,BlockEntity*>::iterator iter2;
  vector<Block3D*>::const_iterator iter;
  vector<Handle>::const_iterator iter1;
  vector<BlockEntity*>::const_iterator iter3;
  map<BlockEntity*,AdjacentInfo>::iterator iter4;
  // XXX FIXME TODO  The code below introduces a bit of a circular
  // dependency between spatial/spacetime code, and the whole learning
  // infrastructure.  It would be much much better if this was designed
  // so that the learning servers figured things out, and then poked
  // whatever stuff was needed into the 3D space.  i.e. so that the
  // learning servers called the spacemap, instead of the spacemap
  // calling the learning servers.
  #ifdef HAVE_PROTOBUF
  // TODO: Send raw block clusters data to learning server
  // construct these adjacent blocks messages to send to the learning server
  learning::message::BlockClusterDataInput dataInput;
  learning::message::BlockClusterStream blockClusters;
  learning::message::BlockClusterLinkStream blockClusterLinks;
  for (iter2 = mBlockEntityList.begin(); iter2 != mBlockEntityList.end(); ++iter2)
  {
  int unitBlockNum = 0;
  BlockEntity* entity = (BlockEntity*)(iter2->second);
  entity->NeighbourBlockEntities.clear();
  vector<Block3D*> blocks = entity->getBlockList();
  for (iter = blocks.begin(); iter != blocks.end(); ++ iter)
  {
  HandleSeq unitBlockHandles = ((Block3D*)(*iter))->getAllMyUnitBlockHandles();
  unitBlockNum +=  unitBlockHandles.size();
  for (iter1 = unitBlockHandles.begin(); iter1 != unitBlockHandles.end(); ++ iter1)
  {
  vector<BlockEntity*>  neighbourEntities = mRootOctree->getNeighbourEntities(mAllUnitAtomsToBlocksMap[(Handle)(*iter1)]);
  for (iter3 = neighbourEntities.begin(); iter3 != neighbourEntities.end(); ++ iter3)
  {
  iter4 = entity->NeighbourBlockEntities.find((BlockEntity*)(*iter3));
  if (iter4 == entity->NeighbourBlockEntities.end())
  {
  AdjacentInfo adjacentInfo;
  adjacentInfo.adjacentUnitNum = 1;
  entity->NeighbourBlockEntities.insert(map<BlockEntity*,AdjacentInfo>::value_type((BlockEntity*)(*iter3),adjacentInfo));
  }
  else
  {
  iter4->second.adjacentUnitNum ++;
  }
  }
  }
  }
  if (this->enableStaticsMapLearning)
  {
  learning::message::BlockCluster* blockCluster = blockClusters.add_blockclusters();
  blockCluster->set_entityid(entity->getEntityID());
  blockCluster->set_blocknumber(unitBlockNum);
  blockCluster->set_blocktype(((BlockMaterial)(((Block3D*)(blocks.front()))->getBlockMaterial())).materialType);
  }
  }
  #endif // HAVE_PROTOBUF
  }
  // get a random near position for building a same blockEntity as the _entity
  // this position should have enough space to build this new entity,
  // not to be overlapping other objects in the map.
  BlockVector Octree3DMapManager::getBuildEntityOffsetPos(BlockEntity* _entity) const
  {
  srand((unsigned)time(0));
  int offsetX = _entity->getBoundingBox().size_x;
  int offsetY = _entity->getBoundingBox().size_y;
  int x, y;
  while(true)
  {
  x = rand() % 5 + offsetX;
  y = rand() % 5 + offsetY;
  // test whether there are enough room for building this entity.
  // todo:
  break;
  }
  BlockVector pos(x,y,0);
  return pos;
  }
*/
