/**
 * MessageSenderAgent.cc
 *
 * Author: Welter Luigi
 */

#include "MessageSenderAgent.h"
#include "PVPSimulator.h"

using namespace PetaverseProxySimulator;

MessageSenderAgent::~MessageSenderAgent() {
}

MessageSenderAgent::MessageSenderAgent() {
}

void MessageSenderAgent::run(opencog::CogServer *server) {
    ((PVPSimulator *) server)->sendMessages();
}

