#ifndef _SPATIAL_ENTITYMANAGER_H
#define _SPATIAL_ENTITYMANAGER_H

#include <algorithm>
#include <map>
#include <set>
#include <vector>

#include <opencog/atomspace/Handle.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>

using namespace opencog;
using namespace opencog::spatial;

namespace opencog
{
    namespace spatial
    {
        class EntityManager
        {
        public:

            EntityManager();
            EntityManager* clone();

            inline Handle getSelfAgentEntity() const {return mSelfAgentEntity;}
            // currently we consider the none block entity has no collision,
            // avatar can get through them
            void addNoneBlockEntity(const Handle& entityNode,
                                    const BlockVector& pos,
                                    bool isSelfObject,
                                    bool isAvatarEntity,
                                    const unsigned long timestamp);
            void removeNoneBlockEntity(const Handle& entityNode);
            bool containsEntity(const Handle& entityNode) const;
            void updateNoneBlockEntityLocation(const Handle& entityNode, BlockVector newpos,
                                               unsigned long timestamp);
            // note that we didn't delete the record
            // when calling removeNoneBlockEntity()
            BlockVector getLastAppearedLocation(const Handle& entityHandle) const;
            Handle getEntity(const BlockVector& pos) const;
            
            template<typename Out>
                Out findAllEntities(Out out) const
            {
                // only calculate the non-block entities and block entities, no including the blocks
                std::vector<Handle> EntityList;

                for ( auto it = mAllNoneBlockEntities.begin(); 
                      it != mAllNoneBlockEntities.end(); ++it ) {
                    EntityList.push_back(*it);
                } // for

                return std::copy(EntityList.begin(), EntityList.end(), out);
            }

            //preserve old interface
            template<typename Out>
                Out getAllObjects(Out out) const
            {
                return findAllEntities(out);
            }

        private:

            Handle mSelfAgentEntity;
            set<Handle> mAllNoneBlockEntities;
            set<Handle> mAllAvatarList;
            multimap<BlockVector, Handle> mPosToNoneBlockEntityMap;
            //Note that even the entity isn't on the map, we still record it here
            map< Handle, vector< pair<unsigned long,BlockVector> > > mNoneBlockEntitieshistoryLocations;
            void _addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp);
            EntityManager(const EntityManager& rhs);
        };
    }
}

#endif
