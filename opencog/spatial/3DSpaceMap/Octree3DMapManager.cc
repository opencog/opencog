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

#include "Octree3DMapManager.h"
#include <iterator>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include "BlockEntity.h"
#include "Entity3D.h"
#include <opencog/learning/statistics/LearningMessage.info.pb.h>

using namespace opencog;
using namespace opencog::spatial;

vector<Handle> Octree3DMapManager::newDisappearBlockEntityList;
vector<BlockEntity*> Octree3DMapManager::newAppearBlockEntityList;
vector<BlockEntity*> Octree3DMapManager::updateBlockEntityList;
vector<BlockEntity*> Octree3DMapManager::updateSuperBlockEntityList;

Octree3DMapManager::Octree3DMapManager(std::string _mapName,int _xMin, int _yMin, int _zMin, int _xDim, int _yDim, int _zDim, int _floorHeight):
    mMapName(_mapName), mFloorHeight(_floorHeight)
{
    // We now allow the whole space not to be a cube (Because our new Unity Embodiment will use real minecraft maps which are usually not cubes)
    // Root octree has a depth of 1, everytime it splits, the depth ++
    // So till the deepest octree every block in it is a unit block
    // So we can calculate the mTotalDepthOfOctree from the sizes of the edges of the map
    // how many unit per edge in this space, indicating the size of the whole space

    mTotalDepthOfOctree = 0;
    int size = 1;

    //default agent height is 1
    mAgentHeight = 1;

    // get the biggest edge among x, y ,z
    int offSet = _xDim;
    if (_yDim > offSet)
        offSet = _yDim;
    if (_zDim > offSet)
        offSet = _zDim;

    while (true)
    {
        size *= 2;
        ++ mTotalDepthOfOctree;
        if (size >= offSet)
            break;
    }

    BlockVector rootPoint(_xMin,_yMin,_zMin);
    mMapBoundingBox.nearLeftBottomConer = rootPoint;
    mMapBoundingBox.size = 0;
    mMapBoundingBox.size_x = _xDim;
    mMapBoundingBox.size_y = _yDim;
    mMapBoundingBox.size_z = _zDim;

    mRootOctree = new Octree(this, rootPoint);

    mAllUnitBlockAtoms.clear();
    mBlockEntityList.clear();
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();

    hasPerceptedMoreThanOneTimes = false;

    // set up the zmq socket to communicate with the learning server
    this->zmqLSContext = new zmq::context_t(1);

    this->socketLSFromLS = new zmq::socket_t(*zmqLSContext, ZMQ_PULL);
    this->socketSendToLS = new zmq::socket_t(*zmqLSContext, ZMQ_PUSH);

    this->toLSIP = config().get("LEARNING_SERVER_PULL_IP");
    this->toLSPort = config().get("LEARNING_SERVER_PULL_PORT");

    this->fromLSIP = config().get("LEARNING_SERVER_PUSH_IP");
    this->fromLSPort = config().get("LEARNING_SERVER_PUSH_PORT");

    string toLSAddress = "tcp://" + this->toLSIP + ":" + this->toLSPort;
    string fromLSAddress = "tcp://" + this->fromLSIP + ":" + this->fromLSPort;

    this->socketSendToLS->connect(toLSAddress.c_str());
    this->socketLSFromLS->connect(fromLSAddress.c_str());

    this->enableStaticsMapLearning = config().get_bool("ENABLE_STATICS_MAP_LEARNING");

}

void Octree3DMapManager::removeAnEntityFromList(BlockEntity* entityToRemove)
{
    map<int,BlockEntity*>::iterator iter2;
    for (iter2 = mBlockEntityList.begin(); iter2 != mBlockEntityList.end(); ++iter2)
    {
        if ((BlockEntity*)(iter2->second) == entityToRemove)
        {
            mBlockEntityList.erase(iter2);
            break;
        }
    }
}

BlockEntity* Octree3DMapManager::findBlockEntityByHandle(const Handle entityNode) const
{
    map<int,BlockEntity*>::const_iterator iter2;
    for (iter2 = mBlockEntityList.begin(); iter2 != mBlockEntityList.end(); ++iter2)
    {
        if (((BlockEntity*)(iter2->second))->mEntityNode == entityNode)
        {
            return (BlockEntity*)(iter2->second);
        }
    }

    return 0;
}

// currently we consider all the none block entities has no collision, agents can get through them
void Octree3DMapManager::addNoneBlockEntity(const Handle &entityNode, BlockVector _centerPosition,
                                            int _width, int _lenght, int _height, double yaw, std::string _entityName, std::string _entityClass,bool is_obstacle)
{
    map<Handle, Entity3D*>::iterator it;
    multimap<BlockVector, Entity3D*>::iterator biter;
    multimap<BlockVector, Entity3D*>::iterator eiter;
    it = mAllNoneBlockEntities.find(entityNode);
    if (it == mAllNoneBlockEntities.end())
    {
        Entity3D* newEntity = new Entity3D(_centerPosition,_width,_lenght,_height,yaw,_entityName,_entityClass, is_obstacle);
        newEntity->mEntityNode = entityNode;
        mAllNoneBlockEntities.insert(map<Handle, Entity3D*>::value_type(entityNode, newEntity));
        mPosToNoneBlockEntityMap.insert(pair<BlockVector, Entity3D*>(_centerPosition,newEntity));
    }
    else
    {
        ((Entity3D*)(it->second))->updateNonBlockEntitySpaceInfo(_centerPosition,_width,_lenght,_height,yaw,is_obstacle);
        biter = mPosToNoneBlockEntityMap.lower_bound(_centerPosition);
        eiter = mPosToNoneBlockEntityMap.upper_bound(_centerPosition);
        while(biter != eiter)
        {
            if (biter->second == it->second)
            {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }

        mPosToNoneBlockEntityMap.insert(pair<BlockVector, Entity3D*>(_centerPosition,(Entity3D*)(it->second)));
    }
}

// currently we consider all the none block entities has no collision, agents can get through them
void Octree3DMapManager::removeNoneBlockEntity(const Handle &entityNode)
{
    map<Handle, Entity3D*>::iterator it;
    it = mAllNoneBlockEntities.find(entityNode);
    if (it != mAllNoneBlockEntities.end())
    {
        Entity3D* entity = mAllNoneBlockEntities[entityNode];

        // delete from mPosToNoneBlockEntityMap first
        BlockVector pos = entity->getPosition();
        multimap<BlockVector, Entity3D*>::iterator biter;
        multimap<BlockVector, Entity3D*>::iterator eiter;
        biter = mPosToNoneBlockEntityMap.lower_bound(pos);
        eiter = mPosToNoneBlockEntityMap.upper_bound(pos);
        while(biter != eiter)
        {
            if (biter->second == entity)
            {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }

        mAllNoneBlockEntities.erase(it);
        delete entity;
    }
}

void Octree3DMapManager:: addSolidUnitBlock(BlockVector _pos, const Handle &_unitBlockAtom, std::string _materialType, std::string _color)
{
    Block3D* block;
    // First, check is there already a block in this position
    if (mRootOctree->checkIsSolid(_pos, block))
        return;


    block = new Block3D(1, _pos, _materialType, _color, _unitBlockAtom);
    mRootOctree->addSolidBlock(block);
    mAllUnitBlockAtoms.insert(map<Handle, BlockVector>::value_type(_unitBlockAtom, _pos));
    mTotalUnitBlockNum ++;

    // when there is not the first time percept the world,
    // adding a new block will cause changes to the blockEntitye
    if (! hasPerceptedMoreThanOneTimes)
        return;

    // first find all the entities will combined to this new block
    vector<BlockEntity*> entities = mRootOctree->getNeighbourEntities(_pos);

    if (entities.size() == 0)
    {
        //there is not any entity will combine with this block
        // so make this block as a new entity which only contains one block
        BlockEntity* entity = new BlockEntity(*block);
        Octree3DMapManager::mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
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
        BlockEntity* entity = new BlockEntity(*block);

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

void Octree3DMapManager::removeSolidUnitBlock(const Handle &blockNode)
{
    map<Handle, BlockVector>::iterator it;
    it = mAllUnitBlockAtoms.find(blockNode);
    if (it == mAllUnitBlockAtoms.end())
    {
        logger().error("Octree3DMapManager::removeSolidUnitBlock: Cannot find this unit block in space map!/n");
    }

    BlockVector _pos = (BlockVector)(it->second);
    Block3D* block;
    // First, check is there a block in this position
    if (! mRootOctree->checkIsSolid(_pos, block))
        return;

    BlockEntity* myEntity = block->mBlockEntity;
    Handle atom = mRootOctree->removeAnUnitSolidBlock(_pos);

    mAllUnitBlockAtoms.erase(atom);
    mTotalUnitBlockNum --;

    if (myEntity == 0)
        return;

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
    BlockVector neighbourPos = mRootOctree->getNeighbourSolidBlockVector(_pos, neighbourBlock);

    if (neighbourPos == BlockVector::ZERO || neighbourBlock == 0)
        return;

    // find from this neighbour BlockVector all the combining blocks
    vector<Block3D*> neighbourblockList = mRootOctree->findAllBlocksCombinedWith(&neighbourPos);
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
        BlockEntity* entity = new BlockEntity(neighbourblockList);
        Octree3DMapManager::mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));

        // get all the neighbour solid positions of this removed unit block
        vector<BlockVector> allNeighbours = mRootOctree->getAllNeighbourSolidBlockVectors(_pos);
        vector<BlockVector>::iterator it = allNeighbours.begin();

        Block3D* curBlock;
        for (; it != allNeighbours.end(); ++it)
        {
            // get the block in this position
            if (! mRootOctree->checkIsSolid((BlockVector&)(*it), curBlock))
                continue;

            if (curBlock->mBlockEntity != 0)
                continue;

            // calculate from this block to find all the blocks combine with it
            vector<Block3D*> newBlockList = mRootOctree->findAllBlocksCombinedWith(&((BlockVector)(*it)));

            // create a new entity for this part of original entity
            BlockEntity* newEntity = new BlockEntity(newBlockList);
            newEntity->addBlocks(newBlockList);

            mBlockEntityList.insert(map<int,BlockEntity*>::value_type(newEntity->getEntityID(), newEntity));
        }

        // remove the original entity
        removeAnEntityFromList(myEntity);
        delete myEntity;
        return;

    }
}

BlockEntity* Octree3DMapManager::findAllBlocksInBlockEntity(BlockVector& _pos)
{
    Block3D* block;
    // First, check is there a block in this position
    if (! mRootOctree->checkIsSolid(_pos, block))
        return 0;

    // check this block has been already found in an existing Entity
    if (block == 0)
        return 0;

    if (block->mBlockEntity != 0)
    {
        // We have ever calculate this BlockEntity, we calculate again
        block->mBlockEntity->clearAllBlocks();

        vector<Block3D*> blockList = mRootOctree->findAllBlocksCombinedWith(&_pos);
        blockList.push_back(block);
        block->mBlockEntity->addBlocks(blockList);

        // entity->SortBlockOrder();

        return block->mBlockEntity;
    }
    else
    {
        BlockEntity* entity = new BlockEntity(*block);

        // find all the blocks combine with this block
         vector<Block3D*> blockList = mRootOctree->findAllBlocksCombinedWith(&_pos);
         entity->addBlocks(blockList);
        // entity->SortBlockOrder();

        Octree3DMapManager::mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
        return entity;
    }
}

void Octree3DMapManager::findAllBlockEntitiesOnTheMap()
{
    map<Handle, BlockVector>::iterator it;
    BlockVector pos;
    Block3D* block;

    int newEntitiesNum = 0;

    for (it =mAllUnitBlockAtoms.begin(); it != mAllUnitBlockAtoms.end(); ++ it)
    {
        pos = (BlockVector)(it->second);

        // First, check is there a block in this position
        if (! mRootOctree->checkIsSolid(pos, block))
            return;

        // check this block has been already found in an existing Entity
        if (block == 0)
            return;

        if (block->mBlockEntity == 0)
        {
            BlockEntity* entity = new BlockEntity(*block);

            // find all the blocks combine with this block
             vector<Block3D*> blockList = mRootOctree->findAllBlocksCombinedWith(&pos);
            //  entity->SortBlockOrder();
             entity->addBlocks(blockList);

            Octree3DMapManager::mBlockEntityList.insert(map<int,BlockEntity*>::value_type(entity->getEntityID(), entity));
            newEntitiesNum ++;
        }

    }

    printf("Found %d BlockEntities in total! \n", newEntitiesNum);
}

// Return the blockEntity occupies this postiton
// If there is not a blockEntity here, return 0
BlockEntity* Octree3DMapManager::getEntityInPos(BlockVector& _pos) const
{
    Block3D* block;
    // First, check is there already a block in this position
    if (mRootOctree->checkIsSolid(_pos, block))
        return 0;

    return (block->mBlockEntity);
}

// get a random near position for building a same blockEnity as the _entity
// this position should have enough space to build this new entity,
// not to be overlapping other objects in the map.
BlockVector Octree3DMapManager::getBuildEnityOffsetPos(BlockEntity* _entity) const
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

const Entity3D* Octree3DMapManager::getEntity( const Handle entityNode ) const
{
    // first find in nonblockEnities
    map<Handle, Entity3D*> ::const_iterator it = mAllNoneBlockEntities.find(entityNode);
    if ( it != mAllNoneBlockEntities.end( ) )
    {
        return it->second;
    }
    else
    {
        // then find in blockEntities:
        Entity3D* entity = findBlockEntityByHandle(entityNode);
        if (entity)
            return entity;
    }

    return 0;

}

const Entity3D* Octree3DMapManager::getEntity( std::string entityName) const
{
    // first find in nonblockEnities
    map<Handle, Entity3D*> ::const_iterator it;

    for ( it = mAllNoneBlockEntities.begin( ); it != mAllNoneBlockEntities.end( ); ++it )
    {
        if (((Entity3D*)(it->second))->getEntityName() == entityName)
            return (it->second);
    } // for

    map<int,BlockEntity*>::const_iterator it2;
    for (it2 = mBlockEntityList.begin( ); it2 != mBlockEntityList.end( ); ++it2 )
    {
        if (((BlockEntity*)(it2->second))->getEntityName() == entityName)
            return (it2->second);
    }

    return 0;
}

std::string Octree3DMapManager::getEntityName(Entity3D* entity) const
{
    return entity->getEntityName();
}

BlockVector Octree3DMapManager::getObjectLocation(Handle objNode) const
{
    // first if this object is a block, return the position of this block
    map<Handle, BlockVector>::const_iterator it2;
    it2 = mAllUnitBlockAtoms.find(objNode);
    if (it2 != mAllUnitBlockAtoms.end())
        return (BlockVector)it2->second;

    // if this object handle is not a block, then it should be an entity
    Entity3D* entity = (Entity3D*)(getEntity(objNode));
    if (entity)
        return entity->getPosition();

    // there is not any object for this hanle on the map
    return BlockVector::ZERO;
}

BlockVector Octree3DMapManager::getObjectDirection(Handle objNode) const
{
    // if it's a block, the direction make no sense, so we just use the x direction
    map<Handle, BlockVector>::const_iterator it2;
    it2 = mAllUnitBlockAtoms.find(objNode);
    if (it2 != mAllUnitBlockAtoms.end())
        return BlockVector::X_UNIT;

    Entity3D* entity = (Entity3D*)(getEntity(objNode));
    if (entity)
        return entity->getDirection();
    else
        return BlockVector::ZERO;
}

bool Octree3DMapManager::checkStandable(BlockVector& pos) const
{

    Block3D* block;
    if (! mMapBoundingBox.isUnitBlockInsideMe(pos))
        return false;

    // check if there is any non-block obstacle in this pos
    if (mPosToNoneBlockEntityMap.count(pos) != 0 )
    return false;

    if (mRootOctree->checkIsSolid(pos,block))
        return false;

    if (pos.z <= mFloorHeight)
        return false;

    // because the agent has a height,
    // we should check if there are enough room above this pos for this agent to stand on
    if (mAgentHeight > 1)
    {
        for (int height = 1; height < mAgentHeight; height ++)
        {
            BlockVector blockAbove(pos.x,pos.y,pos.z + height);
            if (mRootOctree->checkIsSolid(blockAbove,block))
                return false;
        }
    }

    // because there are too many blocks for the whole floor, we don't send these floor blocks to opencog
    // so we just check if the pos height is just on the floor.
    if ( (pos.z - mFloorHeight) == 1)
        return true;

    BlockVector under(pos.x,pos.y,pos.z - 1);
    if (mRootOctree->checkIsSolid(under,block))
        return true;

    return false;

}

bool Octree3DMapManager::checkIsSolid(BlockVector& pos)
{
    Block3D* block;
    return (mRootOctree->checkIsSolid(pos, block));
}

bool Octree3DMapManager::checkStandable(int x, int y, int z) const
{
    BlockVector pos(x,y,z);
    return checkStandable(pos);
}

BlockVector Octree3DMapManager::getNearFreePointAtDistance( const BlockVector& position, int distance, const BlockVector& startDirection , bool toBeStandOn) const
{
    int ztimes = 0;
    int z ;
    Block3D* block;

    BlockVector startD(startDirection.x * -1, startDirection.y * -1, 0);

    while (ztimes <3)
    {
        // we'll first search for the grids of the same high, so begin with z = 0,
        // then search for the lower grids (z = -1), then the higher grids (z = 1)
        if (ztimes == 0)
            z = 0;
        else if (ztimes == 1)
            z = -1;
        else
            z = 1;

        ztimes++;

        // we first search at the startdirection, if cannot find a proper position then go on with the complete search
        BlockVector curpos(position.x + startD.x, position.y + startD.y, position.z + z);
        if (toBeStandOn)
        {
            if (checkStandable(curpos))
                return curpos;
        }
        else
        {
            if (! mRootOctree->checkIsSolid(curpos,block))
                return curpos;
        }

        for (int dis = 1;  dis <= distance; dis++)
        {

            for (int x = 0; x <= dis; x++)
            {
                BlockVector curpos(position.x + x, position.y + dis,position.z + z);
                if (toBeStandOn)
                {
                    if (checkStandable(curpos))
                        return curpos;
                }
                else
                {
                    if (! mRootOctree->checkIsSolid(curpos,block))
                        return curpos;
                }
            }

            for (int y = 0; y <= dis; y++)
            {
                BlockVector curpos(position.x + dis, position.y + y,position.z + z);
                if (toBeStandOn)
                {
                    if (checkStandable(curpos))
                        return curpos;
                }
                else
                {
                    if (! mRootOctree->checkIsSolid(curpos,block))
                        return curpos;
                }
            }

        }
    }
    return BlockVector::ZERO;

}


bool Octree3DMapManager::containsObject(std::string & objectName) const
{
    const Entity3D* entity = getEntity(objectName);

    if (entity)
        return true;
    else
        return false;
}

 bool Octree3DMapManager::containsObject(const Handle objectNode) const
 {
     // there are 3 kinds of object on the map: nonbLockEnitities, blocks, and blockEntities,
     // look up in these 3 collection for this object
     Entity3D* en = (Entity3D*)(getEntity(objectNode));
     if (en != 0)
         return true;

     map<Handle, BlockVector>::const_iterator it2;
     it2 = mAllUnitBlockAtoms.find(objectNode);
     if (it2 != mAllUnitBlockAtoms.end())
         return true;

     return false;
 }

 // TOTO: to be finished
 std::string Octree3DMapManager::toString( const Octree3DMapManager& map )
 {
     std::stringstream out;
     out.precision(25);

     out << map.xMin( ) << " " << map.xMax( ) << " " << map.xDim( ) << " ";
     out << map.yMin( ) << " " << map.yMax( ) << " " << map.yDim( ) << " ";
     out << map.getFloorHeight() << " ";

/*
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

 // TODO:
 Octree3DMapManager* Octree3DMapManager::fromString( const std::string& map )
 {
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

     return new Octree3DMapManager("todo", 0,0,0,128,128,128,98);
 }





//todo
 std::vector<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const Entity3D* observer,
                                                        double besideDistance,
                                                        const Entity3D* entityA,
                                                        const Entity3D* entityB ) const
 {
     std::vector<SPATIAL_RELATION> spatialRelations;

//     math::Vector3 pointInA;
//     math::Vector3 pointInB;

//     LimitRelation status;
//     double distance = entityA.distanceTo( entityB, & pointInA, & pointInB, & status );

//     bool computeAsideRelations = false;
//     if ( ( status.relations[0] & 64 ) > 0 &&
//          ( status.relations[1] & 64 ) > 0 &&
//          ( status.relations[2] & 64 ) > 0 ) {
//         // A overlaps B and vice-versa
//         spatialRelations.push_back(INSIDE);
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//         return spatialRelations;
//     }
//     else if ( ( status.relations[0] & 128 ) > 0 &&
//               ( status.relations[1] & 128 ) > 0 &&
//               ( status.relations[2] & 128 ) > 0 ) {
//         // A is inside B
//         spatialRelations.push_back(INSIDE);
//         spatialRelations.push_back(NEAR);
//         return spatialRelations;
//     }
//     else if ( ( status.relations[0] & 256 ) > 0 &&
//               ( status.relations[1] & 256 ) > 0 &&
//               ( status.relations[2] & 256 ) > 0 ) {
//         // A is outside B
//         spatialRelations.push_back(OUTSIDE);
//         spatialRelations.push_back(NEAR);
//     }
//     else if ( ( status.relations[0] & (64|128|512) ) > 0 &&
//               ( status.relations[1] & (64|128|512) ) > 0 &&
//               ( status.relations[2] & (64|128|512) ) > 0 ) {
//         // A is inside B and touching it
//         spatialRelations.push_back(INSIDE);
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//         return spatialRelations;
//     }
//     else if ( ( status.relations[0] & (64|256|1024) ) > 0 &&
//               ( status.relations[1] & (64|256|1024) ) > 0 &&
//               ( status.relations[2] & (64|256|1024) ) > 0 ) {
//         // A is outside B but touching it
//         spatialRelations.push_back(OUTSIDE);
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//     }
//     else if ( ( status.relations[0] & (1|2) ) == 0 &&
//               ( status.relations[1] & (1|2) ) == 0 &&
//               ( status.relations[2] & (1|2) ) == 0 ) {
//         // A is not completely inside B or vice-versa, but they intersect
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//     }
//     else if ( ( status.relations[0] & (1|2|16|32) ) == 0 &&
//               ( status.relations[1] & (1|2|16|32) ) == 0 &&
//               ( status.relations[2] & 32 ) > 0 ) {
//         // A is on top of B
//         spatialRelations.push_back(ON_TOP_OF);
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//     }
//     else if ( ( ( ( status.relations[0] & (16|32) ) > 0 &&
//                   ( status.relations[1] & (1|2) ) == 0
//                 ) ||
//                 ( ( status.relations[0] & (1|2) ) == 0 &&
//                   ( status.relations[1] & (16|32) ) > 0
//                 )
//               ) &&
//               ( status.relations[2] & (1|2) ) == 0
//             ) {
//         // A is adjacent to B
//         spatialRelations.push_back(ADJACENT);
//         spatialRelations.push_back(TOUCHING);
//         spatialRelations.push_back(NEAR);
//     }
//     else {
//         computeAsideRelations = true;
//     }// if

//     ///*************************** WARNING *********************************////
//     // TODO: UP AXIS = Y (TODO: customize it)
//     //       an intersection must occur at X and Y besides
//     if ( ( status.relations[0] & (1|2) ) == 0 &&
//          ( status.relations[1] & (1|2) ) == 0 ) {
//         if ( ( status.relations[2] & (1|4|16) ) > 0 ) {
//             spatialRelations.push_back(BELOW);
//         }
//         else if ( ( status.relations[2] & (2|8|32) ) > 0 ) {
//             spatialRelations.push_back(ABOVE);
//         }
//     }// if
//     ///*************************** WARNING *********************************////

//     if ( distance > besideDistance ) {
//         spatialRelations.push_back(FAR_);
//         return spatialRelations;
//     }
//     else if ( distance < besideDistance * (LocalSpaceMap2D::NEAR_FACTOR/LocalSpaceMap2D::NEXT_FACTOR) ) {
//         spatialRelations.push_back(NEAR);
//     }
//     else {
//         spatialRelations.push_back(BESIDE);
//     }// if

//     if ( !computeAsideRelations ) {
//         return spatialRelations;
//     }

//     const math::Vector3& observerPosition = observer.getPosition( );

//     math::Vector3 observerDirection;
//     math::Vector3 objectDirection( pointInB - pointInA ); // direction vector from A (this) to B

//     bool observerBetweenObjects = false;

//     if ( observer.getName( ) == entityA.getName( ) ||
//          entityA.getBoundingBox( ).isInside( observerPosition ) ) {
//         observerDirection = (observer.getDirection( ) * objectDirection.length( )+1.0);
//     }
//     else if ( observer.getName( ) == entityB.getName( ) ||
//               entityB.getBoundingBox( ).isInside( observerPosition ) ) {
//         observerDirection = -(observer.getDirection( ) * objectDirection.length( )+1.0);
//     }
//     else {
//         math::Vector3 observerToEntityA, observerToEntityB;
//         {
//             math::Vector3 observerPoint, entityPoint;
//             observer.distanceTo( entityA, &observerPoint, &entityPoint );
//             observerToEntityA = entityPoint - observerPoint; // direction vector from observer to A (this)
//             observerDirection = observerPoint - entityPoint; // direction vector from A (this) to observer
//         }
//         {
//             math::Vector3 observerPoint, entityPoint;
//             observer.distanceTo( entityB, &observerPoint, &entityPoint );
//             observerToEntityB = entityPoint - observerPoint; // direction vector from observer to B (this)
//         }
//         observerToEntityA.normalise( );
//         observerToEntityB.normalise( );

//         double angle = std::acos( observerToEntityA.dotProduct( observerToEntityB ) );
//         observerBetweenObjects = ( std::abs(angle) > 150.0/180.0*M_PI );
//     }// if

//     double distanceToA = observerDirection.length( );
//     double distanceBetweenAandB = objectDirection.length( );

//     observerDirection.normalise(); // direction vector from A (this) to observer
//     objectDirection.normalise();   // direction vector from A (this) to B

//     double angle;
//     {
//         ///*************************** WARNING *********************************////
//         // TODO: UP AXIS = Z (TODO: customize it)

//         // Angle from observerDirection (A to observer) to objectDirection (A to B)
//         angle = std::atan2( objectDirection.y, objectDirection.x ) -
//                 std::atan2( observerDirection.y, observerDirection.x );

//         if ( angle > M_PI ) {
//             angle -= M_PI*2.0;
//         } else if ( angle < -M_PI ) {
//             angle += M_PI*2.0;
//         }
//         ///*************************** WARNING *********************************////
//     }
//     angle *= 180.0/M_PI;

//     double lowerLimit = 20.0;
//     double upperLimit = 110.0;

//     if ( angle > lowerLimit && angle <= upperLimit ) {
//         spatialRelations.push_back( LEFT_OF );
//     }
//     else if ( ( angle > upperLimit && angle <= 180.0 ) ||
//               ( angle >= -180.0 && angle <= -upperLimit ) ) {
//         spatialRelations.push_back( observerBetweenObjects ? BEHIND : IN_FRONT_OF );
//     }
//     else if ( angle > -upperLimit && angle <= -lowerLimit ) {
//         spatialRelations.push_back( RIGHT_OF );
//     }
//     else {
//         if ( distanceToA > distanceBetweenAandB ) {
//             spatialRelations.push_back( observerBetweenObjects ? IN_FRONT_OF : BEHIND );
//         }
//         else {
//             spatialRelations.push_back( angle > 0 ? RIGHT_OF : LEFT_OF );
//         }
//     }// if

//     // BESIDE = next
//     // NEAR = near

     return spatialRelations;
 }

 std::vector<SPATIAL_RELATION> Octree3DMapManager::computeSpatialRelations( const Entity3D* observer,
                                                        double besideDistance,
                                                        const Entity3D* entityA,
                                                        const Entity3D* entityB,
                                                        const Entity3D* entityC ) const
 {
     std::vector<SPATIAL_RELATION> spatialRelations;
     //todo

     return spatialRelations;
 }


 std::string Octree3DMapManager::spatialRelationToString( SPATIAL_RELATION relation ) {
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

 void Octree3DMapManager::computeAllAdjacentBlockClusters()
 {
     map<int,BlockEntity*>::iterator iter2;
     vector<Block3D*>::const_iterator iter;
     vector<Handle>::const_iterator iter1;
     vector<BlockEntity*>::const_iterator iter3;
     map<BlockEntity*,AdjacentInfo>::const_iterator iter4;

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
                vector<BlockEntity*>  neighbourEntities = mRootOctree->getNeighbourEntities(mAllUnitBlockAtoms[(Handle)(*iter1)]);
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
                        ((AdjacentInfo)(iter4->second)).adjacentUnitNum ++;
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

 }

