/*
 * opencog/guile/GenericModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include "GenericModule.h"

#include <opencog/guile/GenericSmob.h>
#include <opencog/guile/GenericSocket.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/NetworkServer.h>
#include <opencog/util/Config.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                   { return GenericModule::id(); }
extern "C" Module*     opencog_module_load()                 { return new GenericModule(); }
extern "C" void        opencog_module_unload(Module* module) { delete module; }

GenericModule::GenericModule() : _port(DEFAULT_PORT)
{
    logger().debug("[GenericModule] constructor");
    if (config().has("GENERIC_PORT"))
        _port = config().get_int("GENERIC_PORT");
    if (config().has("GENERIC_PROMPT"))
        _shell.normal_prompt = config()["GENERIC_PROMPT"];
    if (config().has("GENERIC_PENDING_PROMPT"))
        _shell.pending_prompt = config()["GENERIC_PENDING_PROMPT"];
}

GenericModule::~GenericModule()
{
    logger().debug("[GenericModule] destructor");
    CogServer& cogserver = static_cast<CogServer&>(server());
    NetworkServer& ns = cogserver.networkServer();
    ns.removeListener(_port);
}

void GenericModule::init()
{
    logger().debug("[GenericModule] init");
    // the socket instantiation must be done in the 'init' method (instead of
    // the constructor) because the GenericSocket constructor accesses the
    // GenericModule object
    CogServer& cogserver = static_cast<CogServer&>(server());
    NetworkServer& ns = cogserver.networkServer();
    ns.addListener<GenericSocket>(_port);
}

GenericShell* GenericModule::shell()
{
    return &_shell;
}
