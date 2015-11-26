#ifndef _SPATIAL_ENTITYMANAGER_H
#define _SPATIAL_ENTITYMANAGER_H

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
            void updateNoneBlockEntityLocation(const Handle& entityNode, BlockVector newpos,
                                               unsigned long timestamp);
            // note that we didn't delete the record
            // when calling removeNoneBlockEntity()
            BlockVector getLastAppearedLocation(const Handle& entityHandle) const;
            Handle getEntity(const BlockVector& pos) const;

        private:

            Handle mSelfAgentEntity;
            set<Handle> mAllNoneBlockEntities;
            set<Handle> mAllAvatarList;
            multimap<BlockVector, Handle> mPosToNoneBlockEntityMap;
            map< Handle, vector< pair<unsigned long,BlockVector> > > mNoneBlockEntitieshistoryLocations;
            void _addNonBlockEntityHistoryLocation(Handle entityHandle, BlockVector newLocation, unsigned long timestamp);
            EntityManager(const EntityManager& rhs);
        };
    }
}

#endif
