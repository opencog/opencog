/*
 * opencog/server/BaseServer.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_BASE_SERVER_H
#define _OPENCOG_BASE_SERVER_H

#include <opencog/spatial/space_server/SpaceServer.h>
#include <opencog/spatial/space_server/TimeServer.h>

namespace opencog
{

class AtomSpace;
class BaseServer; 

typedef BaseServer* ServerFactory(void);

/**
 * This class implements a base server class that provides basic functionality
 * for third parties who might wish to use their custom server.
 * There are only two primitives provided by this class: an atomspace instance
 * and a factory method.
 *
 * Applications extending the BaseServer must ensure that:
 *      1. the derived server overrides the factory method 'createInstance'
 *      2. the first call to the 'opencog::server()' global function explicitly
 *         passes the derived server's factory method as parameter.
 *
 * See the files CogServerMain.cc, CogServerh and CogServercc for examples.
 */
class BaseServer
{

protected:

    static AtomSpace* atomSpace;
    static SpaceServer* spacer;
    static TimeServer* timeser;

public:

    /** Returns the atomspace instance. */
    static AtomSpace& getAtomSpace();
    static SpaceServer& getSpaceServer();
    static TimeServer& getTimeServer();

    /** Returns a new BaseServer instance. */
    static BaseServer* createInstance(void);

    BaseServer(void);
    virtual ~BaseServer(void);
}; // class

// singleton instance (following meyer's design pattern)
BaseServer& server(ServerFactory* = BaseServer::createInstance);

}  // namespace

#endif // _OPENCOG_BASE_SERVER_H
