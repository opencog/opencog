/**
 * ImportanceDecayTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 11:30:53 BRT 2007
 */

#include "ImportanceDecayTask.h"
#include "OPC.h"

using namespace OperationalPetController;

ImportanceDecayTask::~ImportanceDecayTask() {
}

ImportanceDecayTask::ImportanceDecayTask() {
    lastTickTime = 0;

    // set task to its active state
    setTaskActive(true);
}

void ImportanceDecayTask::run(MessagingSystem::NetworkElement *ne) {

    // task not active, do not execute its actions
    if(!isTaskActive()){
        return;
    }


    MAIN_LOGGER.log(LADSUtil::Logger::FINE, 
                    "ImportanceDecayTask - Executing decayShortTermImportance().");
    ((OPC *) ne)->decayShortTermImportance();

}
