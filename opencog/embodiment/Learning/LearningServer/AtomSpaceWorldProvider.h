#ifndef _ATOMSPACE_WORLD_PROVIDER_H_
#define _ATOMSPACE_WORLD_PROVIDER_H_

#include "WorldProvider.h"
#include <opencog/atomspace/AtomSpace.h>


class AtomSpaceWorldProvider : public WorldProvider {

    SpaceServer& spaceServer;

    public:
        AtomSpaceWorldProvider(SpaceServer& _spaceServer): spaceServer(_spaceServer) {}
        unsigned long getLatestSimWorldTimestamp() const {
            return spaceServer.getAtomSpace().getTimeServer().getLatestTimestamp();
        }
        AtomSpace& getAtomSpace() const {
            return spaceServer.getAtomSpace();
        }
        SpaceServer& getSpaceServer() const {
        	return spaceServer;
        }
};

#endif // _ATOMSPACE_WORLD_PROVIDER_H_

