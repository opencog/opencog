/**
 * MessageCentral.cc
 *
 * $Header$
 *
 * Author: Elvys Borges
 * Creation: 
 */

#include <stdio.h>
#include "MessageCentral.h"
#include "MessagingSystemExceptions.h"
#include "StringMessage.h"
#include "util/exceptions.h"

#include <LoggerFactory.h>

namespace MessagingSystem {

MessageCentral::~MessageCentral() {
	pthread_mutex_destroy(&messageQueueLock);
}

//MessageCentral::MessageCentral(const Control::SystemParameters &params) : parameters(params) {
MessageCentral::MessageCentral() {

    pthread_mutex_init(&messageQueueLock, NULL);

//    Util::Logger::initMainLogger(Control::LoggerFactory::getLogger(this->parameters, this->parameters.get("ROUTER_ID")));
}

void MessageCentral::lockQueue(){
    pthread_mutex_lock(&messageQueueLock);
}

void MessageCentral::unlockQueue(){
    pthread_mutex_unlock(&messageQueueLock);
}
  

}
