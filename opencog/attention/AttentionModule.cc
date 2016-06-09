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

using namespace std;
using namespace opencog;

DECLARE_MODULE(AttentionModule)

AttentionModule::AttentionModule(CogServer& cs) :
    Module(cs)
{
    _cogserver.registerAgent(ForgettingAgent::info().id,          &forgettingFactory);
    _cogserver.registerAgent(StochasticImportanceDiffusionAgent::info().id,&stochasticDiffusionFactory);
    _cogserver.registerAgent(WageCollectionAgent::info().id,&stochasticUpdatingFactory);
    _cogserver.registerAgent(MinMaxSTIUpdatingAgent::info().id,&minMaxSTIUpdatingFactory);
    _cogserver.registerAgent(HebbianCreationAgent::info().id,&hebbianCreationFactory);
    _cogserver.registerAgent(FocusBoundaryUpdatingAgent::info().id,&focusUpdatingFactory);
    _cogserver.registerAgent(HebbianUpdatingAgent::info().id,&hebbianUpdatingFactory);

    _forgetting_agentptr =
        _cogserver.createAgent(ForgettingAgent::info().id, false);
    _stiupdating_agentptr =
        _cogserver.createAgent(WageCollectionAgent::info().id, false);
    _stidiffusion_agentptr =
        _cogserver.createAgent(StochasticImportanceDiffusionAgent::info().id,false);
    _minmaxstiupdating_agentptr =
        _cogserver.createAgent(MinMaxSTIUpdatingAgent::info().id,false);
    _hebbiancreation_agentptr =
        _cogserver.createAgent(HebbianCreationAgent::info().id,false);
    _focusupdating_agentptr =
        _cogserver.createAgent(FocusBoundaryUpdatingAgent::info().id,false);
    _hebbianupdating_agentptr =
        _cogserver.createAgent(HebbianUpdatingAgent::info().id,false);

    do_start_ecan_register();

    addAFConnection =_cogserver.getAtomSpace().AddAFSignal(
                       boost::bind(&AttentionModule::addAFSignalHandler,
                                   this, _1, _2, _3));
}

AttentionModule::~AttentionModule()
{
    logger().debug("[AttentionModule] enter destructor");
    _cogserver.unregisterAgent(ForgettingAgent::info().id);
    _cogserver.unregisterAgent(StochasticImportanceDiffusionAgent::info().id);
    _cogserver.unregisterAgent(WageCollectionAgent::info().id);
    _cogserver.unregisterAgent(MinMaxSTIUpdatingAgent::info().id);
    _cogserver.unregisterAgent(HebbianCreationAgent::info().id);
    _cogserver.unregisterAgent(FocusBoundaryUpdatingAgent::info().id);
    logger().debug("[AttentionModule] exit destructor");

    do_start_ecan_unregister();

    addAFConnection.disconnect();
}

void AttentionModule::init()
{
}

string AttentionModule::do_start_ecan(Request *req,list<string> args)
{
    //The folowing 3 Agents will all be part of the same thread
    //as the don't need to run that often.
    _cogserver.startAgent(_forgetting_agentptr,true,"attention");
    _cogserver.startAgent(_minmaxstiupdating_agentptr,true,"attention");
    _cogserver.startAgent(_focusupdating_agentptr,true,"attention");

    //The following 4 Agents are all started in their own threads
    _cogserver.startAgent(_stiupdating_agentptr,true,"siua");
    _cogserver.startAgent(_stidiffusion_agentptr,true,"sida");
    _cogserver.startAgent(_hebbiancreation_agentptr,true,"hca");
    _cogserver.startAgent(_hebbianupdating_agentptr,true,"hua");
    return "started ecan\n";
}

void AttentionModule::pause_ecan()
{
    _cogserver.stopAgent(_forgetting_agentptr);
    _cogserver.stopAgent(_minmaxstiupdating_agentptr);
    _cogserver.stopAgent(_stiupdating_agentptr);
    _cogserver.stopAgent(_stidiffusion_agentptr);
    _cogserver.stopAgent(_hebbiancreation_agentptr);
    _cogserver.stopAgent(_focusupdating_agentptr);
    _cogserver.stopAgent(_hebbianupdating_agentptr);
}

/*
 * When an atom enters the AttentionalFocus, it is added to a concurrent_queue
 * so that the HebbianCreationAgent know to check it and create HebbianLinks if
 * neccesary
 */
void AttentionModule::addAFSignalHandler(const Handle& source,
                                        const AttentionValuePtr& av_old,
                                        const AttentionValuePtr& av_new)
{
  newAtomsInAV.push(source);
}
