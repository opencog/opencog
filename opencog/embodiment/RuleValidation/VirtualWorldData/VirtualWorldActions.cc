/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldActions.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#include "VirtualWorldActions.h"

using namespace VirtualWorldData;

/* ----------------------------------------------------------------------------
 * AgentAction
 * ----------------------------------------------------------------------------
 */
AgentAction::AgentAction() {}

AgentAction::AgentAction(const std::string & _ag, const std::string & _ac,
                         const std::vector<std::string> & _p) :
        agent(_ag), action(_ac), params(_p) {}

AgentAction & AgentAction::operator=(const AgentAction & agentAction)
{
    agent = agentAction.agent;
    action = agentAction.action;
    params = agentAction.params;

    return *this;
}

/* ----------------------------------------------------------------------------
 * PetSchema
 * ----------------------------------------------------------------------------
 */
PetSchema::PetSchema() {}

PetSchema::PetSchema(const std::string & _s, const std::string & _r,
                     const std::vector<std::string> & _p) :
        schema(_s), result(_s), params(_p) {}

PetSchema & PetSchema::operator=(const PetSchema & petSchema)
{
    schema = petSchema.schema;
    result = petSchema.result;
    params = petSchema.params;

    return *this;
}


