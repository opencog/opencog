/*
 * opencog/cogserver/server/BaseServer.cc
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


static BaseServer* serverInstance = NULL;

BaseServer* BaseServer::createInstance(AtomSpace* as)
{
    OC_ASSERT(0,
        "Accidentally called the base class!\n"
        "To fix this bug, be sure to make the following call in your code:\n"
        "   server(MyServer::MyCreateInstance);\n"
        "maybe this:\n"
        "   server(CogServer::createInstance);\n"
        "depending on which you want.  You only need to do this once,\n"
        "early during the initialization of your program.\n"
    );
    return NULL;
}


// There might already be an atomspace, whose management we should
// take over.  The user can specify this atomspace.
BaseServer::BaseServer(AtomSpace* as)
{
    // We shouldn't get called with a non-NULL atomSpace static global;
    // that's indicative of a missing call to BaseServer::~BaseServer.
    if (atomSpace) {
        throw (RuntimeException(TRACE_INFO,
               "Found non-NULL atomSpace. BaseServer::~BaseServer not called!"));
    }

    if (as) {
        atomSpace = as;
        attentionbank(as);
    }

    // Set this server as the current server.
    set_current_server(this);
}

BaseServer::~BaseServer()
{
    // We are no longer the current server.
    set_current_server(nullptr);
    atomSpace = nullptr;
}

AtomSpace& BaseServer::getAtomSpace()
{
    return *atomSpace;
}

AttentionBank& BaseServer::getAttentionBank()
{
    return attentionbank(atomSpace);
}

BaseServer& opencog::server(BaseServer* (*factoryFunction)(AtomSpace*),
                            AtomSpace* as)
{
    // Create a new instance using the factory function if we don't
    // already have one.
    if (!serverInstance)
        serverInstance = (*factoryFunction)(as);
    
    // Return a reference to our server instance.
    return *serverInstance;
}

void opencog::set_current_server(BaseServer* currentServer)
{
    // Normally used for stack-based server instantiation.

    // Nothing to do if we've already done this.
    if (serverInstance == currentServer)
        return;

    // Should not call this on more than one server at a time.
    if (serverInstance &&
        currentServer != serverInstance &&
        currentServer != NULL ) {
        throw (RuntimeException(TRACE_INFO,
                "Can't create more than one server singleton instance!"));
    }

    // Set the current server. 
    serverInstance = currentServer;
}
