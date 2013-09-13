/*
 * opencog/spacetime/SpaceServerContainer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _SPACE_SERVER_CONTAINER_H
#define _SPACE_SERVER_CONTAINER_H

#include <string>
#include <opencog/atomspace/Handle.h>

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

/**
 * This is an abstract class to provide an interface used by SpaceServer so that
 * its container be notified about space map manipulation inside it.
 */
class SpaceServerContainer
{

public:
    /** Called by SpaceServer when a map was removed */
    virtual void mapRemoved(Handle mapId) = 0;
    /** Called by SpaceServer when a map was marked persistent, what means it
     * must not be removed by the container */
    virtual void mapPersisted(Handle mapId) = 0;
    /** Called by SpaceServer when it needs a string representation of the map
     * id */
    virtual std::string getMapIdString(Handle mapId) = 0;

};

/** @}*/
} // namespace opencog

#endif // _SPACE_SERVER_CONTAINER_H
