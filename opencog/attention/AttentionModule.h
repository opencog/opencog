/*
 * opencog/attention/AttentionModule.h
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

#ifndef _OPENCOG_ATTENTION_MODULE_H
#define _OPENCOG_ATTENTION_MODULE_H

#include <opencog/attention/ForgettingAgent.h>
#include <opencog/attention/scm/StimulationAgent.h>
#include <opencog/attention/StochasticImportanceDiffusionAgent.h>
#include <opencog/attention/RentCollectionAgent.h>
#include <opencog/attention/MinMaxSTIUpdatingAgent.h>
#include <opencog/attention/HebbianCreationAgent.h>
#include <opencog/attention/FocusBoundaryUpdatingAgent.h>
#include <opencog/attention/HebbianUpdatingAgent.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/Agent.h>

using namespace std;

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/**
 * The AttentionModule
 * is resposible for registering all ECAN agents with the CogServer
 * it also provides a console commmand "start-ecan" to the CogServer
 * which can be used to start all the Agents
 */
class AttentionModule : public Module
{

private:
    Factory<StimulationAgent, Agent>  stimulationFactory;
    Factory<ForgettingAgent, Agent> forgettingFactory;
    Factory<StochasticImportanceDiffusionAgent, Agent> stochasticDiffusionFactory;
    Factory<RentCollectionAgent, Agent> stochasticUpdatingFactory;
    Factory<MinMaxSTIUpdatingAgent, Agent> minMaxSTIUpdatingFactory;
    Factory<HebbianCreationAgent, Agent> hebbianCreationFactory;
    Factory<FocusBoundaryUpdatingAgent, Agent> focusUpdatingFactory;
    Factory<HebbianUpdatingAgent, Agent> hebbianUpdatingFactory;

    AgentPtr _forgetting_agentptr;
    AgentPtr _stiupdating_agentptr;
    AgentPtr _stidiffusion_agentptr;
    AgentPtr _minmaxstiupdating_agentptr;
    AgentPtr _hebbiancreation_agentptr;
    AgentPtr _focusupdating_agentptr;
    AgentPtr _hebbianupdating_agentptr;

    boost::signals2::connection addAFConnection;

    void addAFSignal(const Handle& h, const AttentionValuePtr& av_old,
                     const AttentionValuePtr& av_new);
    void addAFSignalHandler(const Handle& h, const AttentionValuePtr& av_old,
                            const AttentionValuePtr& av_new);
public:

   DECLARE_CMD_REQUEST(AttentionModule, "start-ecan", do_start_ecan,
            "Starts main ECAN agents\n",
            "Usage: ecan-start\n",
            false, true)

    static inline const char* id();

    AttentionModule(CogServer&);
    virtual ~AttentionModule();
    virtual void init();

    void pause_ecan();
    void createAgents();
}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_ATTENTION_MODULE_H
