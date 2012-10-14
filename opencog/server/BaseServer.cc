/*
 * opencog/server/BaseServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#include "BaseServer.h"

#include <memory>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

AtomSpace* BaseServer::atomSpace = NULL;
SpaceServer* BaseServer::spacer = NULL;
TimeServer* BaseServer::timeser = NULL;

BaseServer* BaseServer::createInstance()
{
    return new CogServer();
}

BaseServer::BaseServer()
{
}

BaseServer::~BaseServer()
{
}

AtomSpace& BaseServer::getAtomSpace()
{
    return *atomSpace;
}

SpaceServer& BaseServer::getSpaceServer()
{
    return *spacer;
}

TimeServer& BaseServer::getTimeServer()
{
    return *timeser;
}

// create and return static singleton instance
BaseServer& opencog::server(ServerFactory* factoryFunction)
{
    static std::auto_ptr<BaseServer> instance((*factoryFunction)());
    return *instance;
}
