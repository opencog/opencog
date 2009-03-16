/** 
 * LSCmdMessage.cc
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#include "LSCmdMessage.h"
#include <LADSUtil/StringTokenizer.h>

using namespace LearningServerMessages;

/**
 * Constructor and destructor
 */
LSCmdMessage::~LSCmdMessage(){
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to) : 
						   Message(from, to, MessagingSystem::Message::LS_CMD){
	schema.assign("");
	command.assign("");								   	
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to, 
						   const std::string &msg) :
						   Message(from, to, MessagingSystem::Message::LS_CMD){
						   	
	loadPlainTextRepresentation(msg.c_str());
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to,
						   const std::string &command, const std::string &schema) : 
						   Message(from, to, MessagingSystem::Message::LS_CMD){
							   	
	this->schema.assign(schema);
	this->command.assign(command);								   	
}

/**
 * Public methods
 */
const char * LSCmdMessage::getPlainTextRepresentation(){
	message.assign("");
	
	message.append(command);
	message.append(END_TOKEN);
	message.append(schema);

	return message.c_str();	
}

void LSCmdMessage::loadPlainTextRepresentation(const char *strMessage) {
	LADSUtil::StringTokenizer stringTokenizer((std::string)strMessage, 
										  (std::string)END_TOKEN);
	
	command = stringTokenizer.nextToken();
	schema = stringTokenizer.nextToken();
}

void LSCmdMessage::setCommand(const std::string & command){
	this->command.assign(command);
}

const std::string & LSCmdMessage::getCommand(){
	return command;
}
        
void LSCmdMessage::setSchema(const std::string & schema){
	this->schema.assign(schema);
}

const std::string & LSCmdMessage::getSchema(){
	return schema;
}
