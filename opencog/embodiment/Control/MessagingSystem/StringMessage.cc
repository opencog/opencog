/**
 * StringMessage.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 14:19:25 BRT 2007
 */

#include "StringMessage.h"

using namespace MessagingSystem;

// ***********************************************/
// Constructors/destructors

StringMessage::~StringMessage() {
} 

StringMessage::StringMessage(const std::string &from, const std::string &to) : Message(from, to, Message::STRING) {
    message.assign("");
}

StringMessage::StringMessage(const std::string &from, const std::string &to, const std::string &msg) : Message(from, to, Message::STRING) {
    message.assign(msg);
}

StringMessage::StringMessage(const std::string &from, const std::string &to, const char *msg) : Message(from, to, Message::STRING) {
    message.assign(msg);
}

// ***********************************************/
// Overwritten from message

const char *StringMessage::getPlainTextRepresentation() {
    return message.c_str();
}

void StringMessage::loadPlainTextRepresentation(const char *strMessage) {
    message.assign(strMessage);
}
  
// ***********************************************/
// Getters and setters

void StringMessage::setMessage(const std::string &msg) {
    message.assign(msg);
}

void StringMessage::setMessage(const char *msg) {
    message.assign(msg);
}

const std::string &StringMessage::getMessage() {
    return message;
}
