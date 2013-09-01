/*
 * opencog/spacetime/SpaceTime.h
 *
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _EMBODIMENT_SPACETIME_H
#define _EMBODIMENT_SPACETIME_H

#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/server/CogServer.h>

namespace opencog {

// At the moment, this is a cheap hack to provie all-things
// embodiment related with access to spacetime.

/** \addtogroup grp_spacetime
 *  @{
 */
class SpaceTimeCogServer : public CogServer
{

protected:
    static SpaceServer* spacer;
    static TimeServer* timeser;
public:
    SpaceTimeCogServer();
    SpaceServer& getSpaceServer();
    TimeServer& getTimeServer();

    static BaseServer* createInstance(void);
};

SpaceServer& spaceServer();
TimeServer& timeServer();

/** @}*/
} // namepsace opencog

#endif // _EMBODIMENT_SPACETIME_H

