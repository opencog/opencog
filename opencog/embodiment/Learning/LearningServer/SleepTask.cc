/**
 * SleepTask.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jul  4 12:36:43 BRT 2007
 */
#include "SleepTask.h"

using namespace LearningServer;

SleepTask::~SleepTask() {
}

SleepTask::SleepTask() {
}

void SleepTask::run(MessagingSystem::NetworkElement *ls) {
  MAIN_LOGGER.log(opencog::Logger::FINE,
		  "SleepTask - Executing SleepTask.");
  sleep(2);
}
