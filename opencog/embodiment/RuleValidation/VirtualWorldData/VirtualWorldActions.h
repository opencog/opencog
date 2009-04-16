/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldActions.h
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

#ifndef VIRTUAL_WORLD_ACTIONS_H
#define VIRTUAL_WORLD_ACTIONS_H

#include <string>
#include <vector>

namespace VirtualWorldData
{

struct AgentAction {
    std::string agent;
    std::string action;
    std::vector<std::string> params;

    AgentAction();
    AgentAction(const std::string & agent, const std::string & action,
                const std::vector<std::string> & params);

    AgentAction & operator=(const AgentAction & agentAction);

}; // struct AgentAction

struct PetSchema {
    std::string schema;
    std::string result; // should be action_success or action_failure
    std::vector<std::string> params;

    PetSchema();
    PetSchema(const std::string & schema, const std::string & result,
              const std::vector<std::string> & params);

    PetSchema & operator=(const PetSchema & petSchema);
};

}

#endif
