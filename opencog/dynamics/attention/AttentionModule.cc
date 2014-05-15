/*
 * src/server/AttentionModule.cc
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

#include "AttentionModule.h"

#include <opencog/server/CogServer.h>

#include "opencog/dynamics/attention/atom_types.definitions"

using namespace opencog;

DECLARE_MODULE(AttentionModule)

AttentionModule::AttentionModule(CogServer& cs) :
    Module(cs)
{
    _cogserver.registerAgent(ForgettingAgent::info().id,          &forgettingFactory);
    _cogserver.registerAgent(HebbianUpdatingAgent::info().id,     &hebbianFactory);
#ifdef HAVE_GSL
    _cogserver.registerAgent(ImportanceDiffusionAgent::info().id, &diffusionFactory);
#endif
    _cogserver.registerAgent(ImportanceSpreadingAgent::info().id, &spreadingFactory);
    _cogserver.registerAgent(ImportanceUpdatingAgent::info().id,  &updatingFactory);
}

AttentionModule::~AttentionModule()
{
    logger().debug("[AttentionModule] enter destructor");
    _cogserver.unregisterAgent(ForgettingAgent::info().id);
    _cogserver.unregisterAgent(HebbianUpdatingAgent::info().id);
    _cogserver.unregisterAgent(ImportanceSpreadingAgent::info().id);
#ifdef HAVE_GSL
    _cogserver.unregisterAgent(ImportanceDiffusionAgent::info().id);
#endif
    _cogserver.unregisterAgent(ImportanceUpdatingAgent::info().id);
    logger().debug("[AttentionModule] exit destructor");
}

void AttentionModule::init()
{
}

