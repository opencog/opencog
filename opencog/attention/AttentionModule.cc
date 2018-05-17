/*
 * attention/AttentionModule.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Misgana Bayetta && Roman Treutlein
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
#include "AttentionParamQuery.h"

#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/util/Config.h>
#include <opencog/attention/atom_types.h>

using namespace opencog;
using namespace std::placeholders;

concurrent_queue<Handle> opencog::newAtomsInAV;

DECLARE_MODULE(AttentionModule)

AttentionModule::AttentionModule(CogServer& cs) :
    Module(cs)
{
    init();
    do_start_ecan_register();
    do_stop_ecan_register();
    do_list_ecan_param_register();
    do_set_ecan_param_register();
}

AttentionModule::~AttentionModule()
{
    logger().debug("[AttentionModule] enter destructor");

    _cogserver.unregisterAgent(AFImportanceDiffusionAgent::info().id);
    _cogserver.unregisterAgent(WAImportanceDiffusionAgent::info().id);

    _cogserver.unregisterAgent(ForgettingAgent::info().id);
    _cogserver.unregisterAgent(HebbianUpdatingAgent::info().id);
    _cogserver.unregisterAgent(HebbianCreationAgent::info().id);

    do_start_ecan_unregister();
    do_stop_ecan_unregister();
    do_list_ecan_param_unregister();
    do_set_ecan_param_unregister();

    _cogserver.getAttentionBank().AddAFSignal().disconnect(addAFConnection);

    logger().debug("[AttentionModule] exit destructor");
}

void AttentionModule::init()
{
    AttentionParamQuery _atq(&_cogserver.getAtomSpace());
    _atq.load_default_values(); // Load default ECAN param values into AS

    // Set params
    int af_size = std::stoi(_atq.get_param_value(AttentionParamQuery::af_max_size));
    attentionbank(&_cogserver.getAtomSpace()).set_af_size(af_size);
    
    // New Thread based ECAN agents.
    _cogserver.registerAgent(AFImportanceDiffusionAgent::info().id, &afImportanceFactory);
    _cogserver.registerAgent(WAImportanceDiffusionAgent::info().id, &waImportanceFactory);

    _cogserver.registerAgent(AFRentCollectionAgent::info().id, &afRentFactory);
    _cogserver.registerAgent(WARentCollectionAgent::info().id, &waRentFactory);

    _cogserver.registerAgent(ForgettingAgent::info().id,          &forgettingFactory);
    _cogserver.registerAgent(HebbianCreationAgent::info().id,&hebbianCreationFactory);
    _cogserver.registerAgent(HebbianUpdatingAgent::info().id,&hebbianUpdatingFactory);

    _forgetting_agentptr =
        _cogserver.createAgent(ForgettingAgent::info().id, false);
    _hebbiancreation_agentptr =
        _cogserver.createAgent(HebbianCreationAgent::info().id,false);
    _hebbianupdating_agentptr =
        _cogserver.createAgent(HebbianUpdatingAgent::info().id,false);

    _afImportanceAgentPtr = _cogserver.createAgent(AFImportanceDiffusionAgent::info().id,false);
    _waImportanceAgentPtr = _cogserver.createAgent(WAImportanceDiffusionAgent::info().id,false);

    _afRentAgentPtr = _cogserver.createAgent(AFRentCollectionAgent::info().id, false);
    _waRentAgentPtr = _cogserver.createAgent(WARentCollectionAgent::info().id, false);


    addAFConnection = _cogserver.getAttentionBank().AddAFSignal().connect(
            std::bind(&AttentionModule::addAFSignalHandler,
                this, _1, _2, _3));
}

std::string AttentionModule::do_start_ecan(Request *req, std::list<std::string> args)
{
    std::string afImportance = AFImportanceDiffusionAgent::info().id;
    std::string waImportance = WAImportanceDiffusionAgent::info().id;

    std::string afRent = AFRentCollectionAgent::info().id;
    std::string waRent = WARentCollectionAgent::info().id;

    _cogserver.startAgent(_afImportanceAgentPtr, true, afImportance);
    _cogserver.startAgent(_waImportanceAgentPtr, true, waImportance);

    _cogserver.startAgent(_afRentAgentPtr, true, afRent);
    _cogserver.startAgent(_waRentAgentPtr, true, waRent);

   // _cogserver.startAgent(_forgetting_agentptr,true,"attention");

   // _cogserver.startAgent(_hebbiancreation_agentptr,true,"hca");
   // _cogserver.startAgent(_hebbianupdating_agentptr,true,"hua");

    return ("Started the following agents:\n" + afImportance + "\n" + waImportance +
           "\n" + afRent + "\n" + waRent + "\n");
}

std::string AttentionModule::do_stop_ecan(Request *req, std::list<std::string> args)
{
    _cogserver.stopAgent(_afImportanceAgentPtr);
    _cogserver.stopAgent(_waImportanceAgentPtr);

    _cogserver.stopAgent(_afRentAgentPtr);
    _cogserver.stopAgent(_waRentAgentPtr);

    _cogserver.stopAgent(_forgetting_agentptr);

    _cogserver.stopAgent(_hebbiancreation_agentptr);
    _cogserver.stopAgent(_hebbianupdating_agentptr);

    return "Stopped ECAN agents.\n";
}

std::string AttentionModule::do_list_ecan_param(Request *req, std::list<std::string> args)
{
    std::string response = "";
    AttentionParamQuery _atq(&_cogserver.getAtomSpace());
    HandleSeq hseq = _atq.get_params();
    for(const Handle& h : hseq){
        std::string param = h->get_name();
        response += param + "= " + _atq.get_param_value(param) + "\n"; 
    }
    return response;
}

std::string AttentionModule::do_set_ecan_param(Request *req, std::list<std::string> args)
{
    AttentionParamQuery _atq(&_cogserver.getAtomSpace());
    auto it = args.begin();
    std::string param = *it;
    std::advance(it,1);
    _atq.set_param(param, *it);

   return param+"= "+_atq.get_param_value(param)+"\n";
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
