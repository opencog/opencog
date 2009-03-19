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
using namespace opencog;

SleepTask::~SleepTask() {
}

SleepTask::SleepTask() {
}

void SleepTask::run(MessagingSystem::NetworkElement *ls) {
    logger().log(opencog::Logger::FINE,
                 "SleepTask - Executing SleepTask.");
  sleep(2);
}
