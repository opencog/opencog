/*
 * attention/AttentionModule.cc
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

#include <opencog/cogserver/server/CogServer.h>

#include "opencog/attention/atom_types.definitions"

using namespace opencog;

DECLARE_MODULE(AttentionModule)

AttentionModule::AttentionModule(CogServer& cs) :
    Module(cs)
{
    _cogserver.registerAgent(ForgettingAgent::info().id,          &forgettingFactory);
    _cogserver.registerAgent(StochasticImportanceDiffusionAgent::info().id,&stochasticDiffusionFactory);
    _cogserver.registerAgent(StochasticImportanceUpdatingAgent::info().id,&stochasticUpdatingFactory);
    _cogserver.registerAgent(MinMaxSTIUpdatingAgent::info().id,&minMaxSTIUpdatingFactory);
    _cogserver.registerAgent(HebbianCreationAgent::info().id,&hebbianCreationFactory);
}

AttentionModule::~AttentionModule()
{
    logger().debug("[AttentionModule] enter destructor");
    _cogserver.unregisterAgent(ForgettingAgent::info().id);
    _cogserver.unregisterAgent(StochasticImportanceDiffusionAgent::info().id);
    _cogserver.unregisterAgent(StochasticImportanceUpdatingAgent::info().id);
    _cogserver.unregisterAgent(MinMaxSTIUpdatingAgent::info().id);
    _cogserver.unregisterAgent(HebbianCreationAgent::info().id);
    logger().debug("[AttentionModule] exit destructor");
}

void AttentionModule::init()
{
}

void AttentionModule::createAgents()
{
    printf("1");
    _forgetting_agentptr =
        _cogserver.createAgent(ForgettingAgent::info().id, false);
    printf("2");
    _stiupdating_agentptr =
        _cogserver.createAgent(StochasticImportanceUpdatingAgent::info().id, false);
    _stidiffusion_agentptr =
        _cogserver.createAgent(StochasticImportanceDiffusionAgent::info().id,false);
    _minmaxstiupdating_agentptr =
        _cogserver.createAgent(MinMaxSTIUpdatingAgent::info().id,false);
    _hebbiancreation_agentptr =
        _cogserver.createAgent(HebbianCreationAgent::info().id,false);
}

void AttentionModule::start_ecan()
{
    _cogserver.startAgent(_forgetting_agentptr);
    _cogserver.startAgent(_minmaxstiupdating_agentptr);
    _cogserver.startAgent(_stiupdating_agentptr,true,"siua");
    _cogserver.startAgent(_stidiffusion_agentptr,true,"sida");
    _cogserver.startAgent(_hebbiancreation_agentptr,true,"hca");
}

void AttentionModule::pause_ecan()
{
    _cogserver.stopAgent(_forgetting_agentptr);
    _cogserver.stopAgent(_minmaxstiupdating_agentptr);
    _cogserver.stopAgent(_stiupdating_agentptr);
    _cogserver.stopAgent(_stidiffusion_agentptr);
    _cogserver.stopAgent(_hebbiancreation_agentptr);
}


