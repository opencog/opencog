#ifndef VIRTUAL_WORLD_ACTIONS_H
#define VIRTUAL_WORLD_ACTIONS_H

#include <string>
#include <vector>

namespace VirtualWorldData {

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
