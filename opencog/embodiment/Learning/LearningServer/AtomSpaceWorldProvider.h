/*
 * opencog/embodiment/Learning/LearningServer/AtomSpaceWorldProvider.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
 * All Rights Reserved
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

