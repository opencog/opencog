#include "EntityManager.h"

using namespace opencog;
using namespace opencog::spatial;

EntityManager::EntityManager():mSelfAgentEntity(Handle::UNDEFINED)
{
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    mNoneBlockEntitieshistoryLocations.clear();
}

EntityManager::EntityManager(const EntityManager& rhs):
    mSelfAgentEntity(rhs.mSelfAgentEntity)
{
    mAllNoneBlockEntities.clear();
    mPosToNoneBlockEntityMap.clear();
    mAllAvatarList.clear();
    mAllNoneBlockEntities = rhs.mAllNoneBlockEntities;
    mPosToNoneBlockEntityMap = rhs.mPosToNoneBlockEntityMap;
    mAllAvatarList = rhs.mAllAvatarList;
    mNoneBlockEntitieshistoryLocations = rhs.mNoneBlockEntitieshistoryLocations;
}

EntityManager* EntityManager::clone()
{
    EntityManager* cloneEntityManager = new EntityManager(*this);
    return cloneEntityManager;
}

// currently we consider all the none block entities has no collision, agents can get through them
void EntityManager::addNoneBlockEntity(const Handle &entityNode, const BlockVector& pos,
                                       bool isSelfObject, bool isAvatarEntity, const unsigned long timestamp)
{
    auto it = mAllNoneBlockEntities.find(entityNode);
    if (it == mAllNoneBlockEntities.end()) {
        mAllNoneBlockEntities.insert(entityNode);
        mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(pos, entityNode));
        if(isSelfObject){ mSelfAgentEntity = entityNode;}
        if(isAvatarEntity){ mAllAvatarList.insert(entityNode);}
        _addNonBlockEntityHistoryLocation(entityNode, pos, timestamp);
    }
    else {
        updateNoneBlockEntityLocation(entityNode, pos,timestamp);
    }
}

void EntityManager::_addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp)
{
    auto it = mNoneBlockEntitieshistoryLocations.find(entityHandle);
    if (it == mNoneBlockEntitieshistoryLocations.end()) {
        vector< pair< unsigned long,BlockVector> > newVector;
        newVector.push_back(pair<unsigned long,BlockVector>(timestamp, newLocation));
        mNoneBlockEntitieshistoryLocations.insert(map<Handle, vector< pair<unsigned long, BlockVector> > >::value_type(entityHandle, newVector));
    } else {
        vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if(newLocation == (oneEntityHistories.back()).second) {
            return;
        } else {
            oneEntityHistories.push_back(pair<unsigned long,BlockVector>(timestamp, newLocation));
        }
    }
}
// currently we consider all the none block entities has no collision, agents can get through them

void EntityManager::removeNoneBlockEntity(const Handle& entityNode)
{
    auto it = mAllNoneBlockEntities.find(entityNode);
    if (it != mAllNoneBlockEntities.end()) {
        // delete from mPosToNoneBlockEntityMap first
        BlockVector pos = getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(pos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(pos);
        while (biter != eiter) {
            if (biter->second == entityNode) {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
        mAllNoneBlockEntities.erase(it);
    }
}

bool EntityManager::containsEntity(const Handle& entityNode) const
{
    return mAllNoneBlockEntities.find(entityNode) != mAllNoneBlockEntities.end();
}

void EntityManager::updateNoneBlockEntityLocation(const Handle& entityNode, BlockVector newpos, unsigned long timestamp)
{
    mPosToNoneBlockEntityMap.insert(pair<BlockVector, Handle>(newpos, entityNode));

    if (mAllNoneBlockEntities.find(entityNode) != mAllNoneBlockEntities.end()) {
        BlockVector lastpos = getLastAppearedLocation(entityNode);
        auto biter = mPosToNoneBlockEntityMap.lower_bound(lastpos);
        auto eiter = mPosToNoneBlockEntityMap.upper_bound(lastpos);
        while (biter != eiter) {
            if (biter->second == entityNode) {
                mPosToNoneBlockEntityMap.erase(biter);
                break;
            }
            ++ biter;
        }
    }
    _addNonBlockEntityHistoryLocation(entityNode, newpos, timestamp);
}

BlockVector EntityManager::getLastAppearedLocation(const Handle& entityHandle) const
{
    auto it = mNoneBlockEntitieshistoryLocations.find(entityHandle);
    if (it == mNoneBlockEntitieshistoryLocations.end()) {
        return BlockVector::ZERO;
    } else {
        const vector< pair<unsigned long,BlockVector> >& oneEntityHistories = it->second;
        if (oneEntityHistories.size() == 0) {
            return BlockVector::ZERO;
        } else {
            return (oneEntityHistories.back()).second;
        }
    }
}

Handle EntityManager::getEntity(const BlockVector& pos) const
{
    auto it = mPosToNoneBlockEntityMap.find(pos);
    if (it == mPosToNoneBlockEntityMap.end()) {
        return Handle::UNDEFINED;
    } else {
        return it->second;
    }
}
