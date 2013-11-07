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

#include <memory>
#include <opencog/util/oc_assert.h>
#include "BaseServer.h"

using namespace opencog;

AtomSpace* BaseServer::atomSpace = NULL;

BaseServer* BaseServer::createInstance()
{
    OC_ASSERT(0,
        "Accidentally called the base class!\n"
        "To fix this bug, be sure to make the following call in your code:\n"
        "   server(MyServer::MyCreateInstance);\n"
        "So, for example:\n"
        "   server(OAC::createInstance);\n"
        "or maybe this:\n"
        "   server(CogServer::createInstance);\n"
        "depending on which you want.  You only need to do this once,\n"
        "early during the initialization of your program.\n"
    );
    return NULL;
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

// create and return static singleton instance
BaseServer& opencog::server(BaseServer* (*factoryFunction)())
{
    static std::unique_ptr<BaseServer> instance((*factoryFunction)());
    return *instance;
}
