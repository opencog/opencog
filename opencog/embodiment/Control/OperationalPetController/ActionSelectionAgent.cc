/**
 * ActionSelectionAgent.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:46:02 BRT 2007
 */

#include "OPC.h"
#include "ActionSelectionAgent.h"

using namespace OperationalPetController;

ActionSelectionAgent::~ActionSelectionAgent() {
}

ActionSelectionAgent::ActionSelectionAgent() {
    lastTickTime = 0;
}

void ActionSelectionAgent::run(opencog::CogServer *server) {
    
    logger().log(opencog::Logger::DEBUG, "ActionSelectionAgent - Executing schemaSelection().");
    ((OPC *) server)->schemaSelection();
}
