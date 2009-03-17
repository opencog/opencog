/**
 * RouterMessage.cc
 *
 * Author: Carlos Lopes
 * Creation: Fri Feb 1 2008
 */
#include "RouterMessage.h"

using namespace MessagingSystem;

RouterMessage::~RouterMessage(){
}

RouterMessage::RouterMessage(const std::string &from, const std::string &to, 
                             int _encapsulateType) : Message(from, to, Message::ROUTER){
    encapsulateType = _encapsulateType;
    message.assign("");                                 
}

RouterMessage::RouterMessage(const std::string &from, const std::string &to, 
                             int _encapsulateType, const std::string& msg) 
                             : Message(from, to, Message::ROUTER){
    encapsulateType = _encapsulateType;
    message.assign(msg);
}

const char *  RouterMessage::getPlainTextRepresentation(){
    return message.c_str();
}


void RouterMessage::loadPlainTextRepresentation(const char *strMessage){
    message.assign(strMessage);
}

void RouterMessage::setMessage(const std::string& msg){
    message.assign(msg);
}

const std::string& RouterMessage::getMessage(){
    return message;
}

void RouterMessage::setEncapsulateType(int type){
    encapsulateType = type;
}

int RouterMessage::getEncapsulateType(){
    return encapsulateType;
}

