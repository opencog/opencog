/**
 * ActionSelectionTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:46:02 BRT 2007
 */

#include "ActionSelectionTask.h"
//#include <SimulationParameters.h>
#include "OPC.h"

using namespace OperationalPetController;

ActionSelectionTask::~ActionSelectionTask() {
}

ActionSelectionTask::ActionSelectionTask() {
    lastTickTime = 0;

    // active task execution
    setTaskActive(true);
}

void ActionSelectionTask::run(MessagingSystem::NetworkElement *opc) {
    
    // task not active, do not execute its actions
    if(!isTaskActive()){
        MAIN_LOGGER.log(LADSUtil::Logger::DEBUG, "ActionSelectionTask - Task is no active.");
        return;
    }

    MAIN_LOGGER.log(LADSUtil::Logger::DEBUG, "ActionSelectionTask - Executing schemaSelection().");
    ((OPC *) opc)->schemaSelection();
}
