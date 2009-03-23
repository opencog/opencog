#include "GoldStdMessage.h"

using namespace PetaverseProxySimulator;

GoldStdMessage::GoldStdMessage(unsigned long _timestamp, Message* _message) : 
    timestamp(_timestamp), message(_message) {
}

GoldStdMessage::~GoldStdMessage() {
    delete message;
}

unsigned long GoldStdMessage::getTimestamp() {
    return timestamp;
}

Message* GoldStdMessage::getMessage() {
    return message;
}
