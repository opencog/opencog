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

#include <opencog/attention/experimental/AFImportanceDiffusionAgent.h>
#include <opencog/attention/experimental/AFRentCollectionAgent.h>

#include <opencog/attention/experimental/WAImportanceDiffusionAgent.h>
#include <opencog/attention/experimental/WARentCollectionAgent.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

class ExperimentalAttentionModule : public Module
{

private:  
    
    Factory<AFImportanceDiffusionAgent, Agent>  afImportanceFactory;
    Factory<WAImportanceDiffusionAgent, Agent>  waImportanceFactory;
    
    Factory<AFRentCollectionAgent, Agent>  afRentFactory;
    Factory<WARentCollectionAgent, Agent>  waRentFactory;
   
    AgentPtr _afImportanceAgentPtr; 
    AgentPtr _waImportanceAgentPtr;
    
    AgentPtr _waRentAgentPtr;
    AgentPtr _afRentAgentPtr;

public:
    
    DECLARE_CMD_REQUEST(ExperimentalAttentionModule, "start-ecan", do_start_ecan,
                        "Starts main ECAN agents\n",
                        "Usage: ecan-start\n", false, true)
 
    static inline const char* id();

    ExperimentalAttentionModule(CogServer&);
    virtual ~ExperimentalAttentionModule();
    virtual void init();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_EXPERMENTAL_ATTENTION_MODULE_H
