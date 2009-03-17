/**
 * SleepTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */

#include "SleepTask.h"
#include "OPC.h"

using namespace OperationalPetController;

SleepTask::~SleepTask() {
}

SleepTask::SleepTask() {
    // set task to its active state
    setTaskActive(true);
}

void SleepTask::run(MessagingSystem::NetworkElement *opc) {
    
    // task not active, do not execute its actions
    if(!isTaskActive()){
        return;
    }

	MAIN_LOGGER.log(LADSUtil::Logger::FINE, "SleepTask - Sleeping for one second.");
    sleep(1);
}
