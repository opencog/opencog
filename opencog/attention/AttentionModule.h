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

#ifndef _OPENCOG_EXPERIMENTAL_ATTENTION_MODULE_H
#define _OPENCOG_EXPERIMENTAL_ATTENTION_MODULE_H

#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/CogServer.h>

#include "AFImportanceDiffusionAgent.h"
#include "AFRentCollectionAgent.h"

#include "WAImportanceDiffusionAgent.h"
#include "WARentCollectionAgent.h"

#include "ForgettingAgent.h"
#include "FocusBoundaryUpdatingAgent.h"
#include "HebbianUpdatingAgent.h"
#include "HebbianCreationAgent.h"

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class AttentionModule : public Module
{

private:

    Factory<AFImportanceDiffusionAgent, Agent>  afImportanceFactory;
    Factory<WAImportanceDiffusionAgent, Agent>  waImportanceFactory;

    Factory<AFRentCollectionAgent, Agent>  afRentFactory;
    Factory<WARentCollectionAgent, Agent>  waRentFactory;

    Factory<ForgettingAgent, Agent> forgettingFactory;

    Factory<FocusBoundaryUpdatingAgent, Agent>  focusUpdatingFactory;

    Factory<HebbianUpdatingAgent, Agent> hebbianUpdatingFactory;
    Factory<HebbianCreationAgent, Agent> hebbianCreationFactory;

    AgentPtr _forgetting_agentptr;

    AgentPtr _minmaxstiupdating_agentptr;

    AgentPtr _focusupdating_agentptr;

    AgentPtr _hebbianupdating_agentptr;
    AgentPtr _hebbiancreation_agentptr;

    AgentPtr _afImportanceAgentPtr;
    AgentPtr _waImportanceAgentPtr;

    AgentPtr _waRentAgentPtr;
    AgentPtr _afRentAgentPtr;

    boost::signals2::connection addAFConnection;

    void addAFSignal(const Handle& h, const AttentionValuePtr& av_old,
                     const AttentionValuePtr& av_new);
    void addAFSignalHandler(const Handle& h, const AttentionValuePtr& av_old,
                            const AttentionValuePtr& av_new);

public:

    DECLARE_CMD_REQUEST(AttentionModule, "start-ecan", do_start_ecan,
                        "Starts  ECAN agents. use agents-active command to view a list of agents started.\n",
                        "Usage: start-ecan\n", false, true)

    DECLARE_CMD_REQUEST(AttentionModule, "stop-ecan", do_stop_ecan,
                        "Stops all active  ECAN agents\n",
                        "Usage: stop-ecan\n", false, true)


    DECLARE_CMD_REQUEST(AttentionModule, "list-ecan-param", do_list_ecan_param,
                        "Lists all ecan parameters and their values.\n",
                        "Usage: list-ecan-params\n", false, true)

    DECLARE_CMD_REQUEST(AttentionModule, "set-ecan-param", do_set_ecan_param,
                        "Sets the value of an ecan parameter\n",
                        "Usage: set-ecan-param <param-name> <param-value> \n", false, true)



    static inline const char* id();

    AttentionModule(CogServer&);
    virtual ~AttentionModule();
    virtual void init();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_EXPERMENTAL_ATTENTION_MODULE_H
