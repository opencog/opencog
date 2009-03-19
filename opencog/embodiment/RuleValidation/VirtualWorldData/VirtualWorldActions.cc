#include "VirtualWorldActions.h"

using namespace VirtualWorldData;

/* ----------------------------------------------------------------------------
 * AgentAction
 * ----------------------------------------------------------------------------
 */
AgentAction::AgentAction(){}

AgentAction::AgentAction(const std::string & _ag, const std::string & _ac, 
                         const std::vector<std::string> & _p) : 
                         agent(_ag), action(_ac), params(_p) {}

AgentAction & AgentAction::operator=(const AgentAction & agentAction){
    agent = agentAction.agent;
    action = agentAction.action;
    params = agentAction.params;

    return *this;
}

/* ----------------------------------------------------------------------------
 * PetSchema
 * ----------------------------------------------------------------------------
 */
PetSchema::PetSchema(){}

PetSchema::PetSchema(const std::string & _s, const std::string & _r, 
                     const std::vector<std::string> & _p) :
                     schema(_s), result(_s), params(_p) {}

PetSchema & PetSchema::operator=(const PetSchema & petSchema){
    schema = petSchema.schema;
    result = petSchema.result;
    params = petSchema.params;

    return *this;
}


