/*
 * AttentionSCM.cc
 * 
 * Author: Misgana Bayetta <misgana.bayettta@gmail.com>
 *
 * Copyright (C) 2015 OpenCog Foundation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_ATTENTION_SCM_H
#define _OPENCOG_ATTENTION_SCM_H

#include "StimulationAgent.h"
#include <opencog/guile/SchemePrimitive.h>

namespace opencog {

    class AttentionSCM
    {
    private:
	static void* init_in_guile(void*);
	static void init_in_module(void*);
	void init(void);
        
	AgentPtr get_agent(const std::string& agent_id);
	
	/**
	 * Stimulates an atom by calling the stimulateAtom method of 
	 * StimulationAgent.
	 * 
	 * @throw RuntimeException Throws exception if StimulationAgent 
	 *        is not running or registered by the cogserver.
	 */
	void stimulate_atom(Handle h, short sti_amount);

	/**
	 * Calls the run method of the ImportanceUpdating and ImportanceSpreading
	 * Agents.
	 * 
	 * @throw RuntimeException Throws exception if ImportanceUpdating and
	 *        ImportanceSpreading agents were not registered by the cogserver.
	 */
	void update_spread_importance(void);

    public:
	AttentionSCM();
    };

    extern "C" {
	void opencog_ecan_init(void);
    };
}
#endif // _OPENCOG_ATTENTION_SCM_H


using namespace opencog;

AttentionSCM::AttentionSCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

void* AttentionSCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog attention", init_in_module, self);
    scm_c_use_module("opencog attention");
    return NULL;
}

void AttentionSCM::init_in_module(void* data)
{
    AttentionSCM* self = (AttentionSCM*) data;
    self->init();
}

void AttentionSCM::init()
{
    define_scheme_primitive("stimulate-atom", &AttentionSCM::stimulate_atom,
			    this, "attention");
    
    define_scheme_primitive("update-spread-importance",
			    &AttentionSCM::update_spread_importance, this,
			    "attention");
}

extern "C" {

    void opencog_ecan_init(void)
    {
	static AttentionSCM stimulation_agent;
    }
};

AgentPtr AttentionSCM::get_agent(const std::string& agent_id)
{
    AgentPtr agentPtr;
    for (AgentPtr agptr : cogserver().runningAgents()) {
	if (agptr->classinfo().id == agent_id) {
	    agentPtr = agptr;
	    break;
	}
    }
    
    return agentPtr;
}

void AttentionSCM::stimulate_atom(Handle h, short sti_amount)
{
    // Get the AgentPtr to StimulationAgent and stimulate h via the pointer.
    std::string stimulation_agent_id = "opencog::StimulationAgent";
    auto stimulation_agentptr = get_agent(stimulation_agent_id);

    if (stimulation_agentptr) {
	stimulation_agentptr->stimulateAtom(h, sti_amount);
    } else {
	throw (RuntimeException(TRACE_INFO,
				("Unable to get a reference to" +
				stimulation_agent_id +
				"Make sure agent is running.").c_str())
	);
    }
}

void AttentionSCM::update_spread_importance()
{
    // Get the AgentPtr to ImportanceUpdating and ImportanceSpreading agents
    // and invoke their run method.
    std::string updating_agent_id = "opencog::ImportanceUpdatingAgent";
    std::string spreading_agent_id = "opencog::SimpleImportanceDiffusionAgent";

    auto updating_agentptr = get_agent(updating_agent_id);
    auto spreading_agentptr = get_agent(spreading_agent_id);

    if (updating_agentptr and spreading_agentptr) {
	// Order of execution matters here. updatingAgent first followed by 
	// SpreadingAgent.
	updating_agentptr->run();
	spreading_agentptr->run();

    } else {
	throw (RuntimeException(TRACE_INFO,
				("Unable to get a reference to either " +
				updating_agent_id + " or " + spreading_agent_id
				+ ". Make sure both agents are running.").c_str()
				)
	);
    }
}
