/*
 * opencog/server/BuiltinRequests.cc
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

#include "BuiltinRequests.h"

#include <opencog/server/CogServer.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                   { return BuiltinRequests::id(); }
extern "C" Module*     opencog_module_load()                 { return new BuiltinRequests(); }
extern "C" void        opencog_module_unload(Module* module) { delete module; }

BuiltinRequests::BuiltinRequests()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(ListRequest::info().id,         &listFactory); 
    cogserver.registerRequest(LoadRequest::info().id,         &loadFactory);
    cogserver.registerRequest(DataRequest::info().id,         &dataFactory); 
    cogserver.registerRequest(HelpRequest::info().id,         &helpFactory); 
    cogserver.registerRequest(ExitRequest::info().id,         &exitFactory); 
    cogserver.registerRequest(ShutdownRequest::info().id,     &shutdownFactory); 
    cogserver.registerRequest(LoadModuleRequest::info().id,   &loadmoduleFactory);
    cogserver.registerRequest(UnloadModuleRequest::info().id, &unloadmoduleFactory);
}

BuiltinRequests::~BuiltinRequests()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterRequest(ListRequest::info().id);
    cogserver.unregisterRequest(LoadRequest::info().id);
    cogserver.unregisterRequest(DataRequest::info().id);
    cogserver.unregisterRequest(HelpRequest::info().id);
    cogserver.unregisterRequest(ExitRequest::info().id);
    cogserver.unregisterRequest(ShutdownRequest::info().id);
    cogserver.unregisterRequest(LoadModuleRequest::info().id);
    cogserver.unregisterRequest(UnloadModuleRequest::info().id);
}

void BuiltinRequests::init()
{
}
