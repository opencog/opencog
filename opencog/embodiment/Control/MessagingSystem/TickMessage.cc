/**
 * TickMessage.cc
 *
 * $Header$
 *
 * Author: Elvys Borges
 * Creation: Wed Feb 14 14:19:25 BRT 2008
 */


#include "TickMessage.h"

using namespace MessagingSystem;

// ***********************************************/
// Constructors/destructors

TickMessage::~TickMessage() {
} 

TickMessage::TickMessage(const std::string &from, const std::string &to) : Message(from, to, Message::TICK) {
}


// ***********************************************/
// Overwritten from message

const char *TickMessage::getPlainTextRepresentation() {
static char* tick = "TICK_MESSAGE";
	return tick;
}

void TickMessage::loadPlainTextRepresentation(const char *strMessage) {

}
  
