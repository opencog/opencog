/*
 * opencog/embodiment/Learning/LearningServer/AtomSpaceWorldProvider.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/embodiment/Learning/behavior/WorldProvider.h>

namespace opencog {

class AtomSpaceWorldProvider : public WorldProvider
{

    AtomSpace& atomSpace;

public:
    AtomSpaceWorldProvider(AtomSpace& _atomSpace): atomSpace(_atomSpace) {}
    unsigned long getLatestSimWorldTimestamp() const {
        return timeServer().getLatestTimestamp();
    }
    AtomSpace& getAtomSpace() const {
        return atomSpace;
    }
};

} // namespace opencog

#endif // _ATOMSPACE_WORLD_PROVIDER_H_

