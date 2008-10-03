/*
 * opencog/guile/SchemeModule.cc
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

#include "SchemeModule.h"

#include <opencog/guile/SchemeSmob.h>
#include <opencog/guile/SchemeSocket.h>
#include <opencog/server/CogServer.h>
#include <opencog/server/NetworkServer.h>
#include <opencog/util/Config.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                   { return SchemeModule::id(); }
extern "C" Module*     opencog_module_load()                 { return new SchemeModule(); }
extern "C" void        opencog_module_unload(Module* module) { delete module; }

SchemeModule::SchemeModule() : _port(DEFAULT_PORT)
{
    logger().debug("[SchemeModule] constructor");
    if (config().has("SCHEME_PORT"))
        _port = config().get_int("SCHEME_PORT");
    if (config().has("SCHEME_PROMPT"))
        _shell.normal_prompt = config()["SCHEME_PROMPT"];
    if (config().has("SCHEME_PENDING_PROMPT"))
        _shell.pending_prompt = config()["SCHEME_PENDING_PROMPT"];
}

SchemeModule::~SchemeModule()
{
    logger().debug("[SchemeModule] destructor");
    CogServer& cogserver = static_cast<CogServer&>(server());
    NetworkServer& ns = cogserver.networkServer();
    ns.removeListener(_port);
}

void SchemeModule::init()
{
    logger().debug("[SchemeModule] init");
    // the socket instantiation must be done in the 'init' method (instead of
    // the constructor) because the SchemeSocket constructor accesses the
    // SchemeModule object
    CogServer& cogserver = static_cast<CogServer&>(server());
    NetworkServer& ns = cogserver.networkServer();
    ns.addListener<SchemeSocket>(_port);
}

SchemeShell* SchemeModule::shell()
{
    return &_shell;
}
