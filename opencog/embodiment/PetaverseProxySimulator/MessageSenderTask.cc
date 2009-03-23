/**
 * MessageSenderTask.cc
 *
 * Author: Welter Luigi
 */

#include "MessageSenderTask.h"
#include "PVPSimulator.h"

using namespace PetaverseProxySimulator;

MessageSenderTask::~MessageSenderTask() {
}

MessageSenderTask::MessageSenderTask() {
}

void MessageSenderTask::run(MessagingSystem::NetworkElement *ne) {
    ((PVPSimulator *) ne)->sendMessages();
}

