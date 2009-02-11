/*
 * src/server/AttentionModule.cc
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

#include "AttentionModule.h"

#include <opencog/server/CogServer.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                   { return AttentionModule::id(); }
extern "C" Module*     opencog_module_load()                 { return new AttentionModule(); }
extern "C" void        opencog_module_unload(Module* module) { delete module; }

AttentionModule::AttentionModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(ForgettingAgent::info().id,          &forgettingFactory);
    cogserver.registerAgent(HebbianUpdatingAgent::info().id,     &hebbianFactory);
#ifdef HAVE_GSL
    cogserver.registerAgent(ImportanceDiffusionAgent::info().id, &diffusionFactory);
#endif
    cogserver.registerAgent(ImportanceSpreadingAgent::info().id, &spreadingFactory);
    cogserver.registerAgent(ImportanceUpdatingAgent::info().id,  &updatingFactory);
    cogserver.registerAgent(STIDecayingAgent::info().id,         &stidecayingFactory);
}

AttentionModule::~AttentionModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterAgent(ForgettingAgent::info().id);
    cogserver.unregisterAgent(HebbianUpdatingAgent::info().id);
    cogserver.unregisterAgent(ImportanceSpreadingAgent::info().id);
#ifdef HAVE_GSL
    cogserver.unregisterAgent(ImportanceDiffusionAgent::info().id);
#endif
    cogserver.unregisterAgent(ImportanceUpdatingAgent::info().id);
    cogserver.unregisterAgent(STIDecayingAgent::info().id);
}

void AttentionModule::init()
{
}
