/** 
 * StopLearningMessage.cc
 * 
 * Author: Erickson Nascimento
 * Copyright(c), 2008
 */

#include <vector>
#include "StopLearningMessage.h"

#include "SpaceServer.h"
#include "util/StringTokenizer.h"

#include <opencog/xml/NMXmlParser.h>
#include <opencog/xml/NMXmlExporter.h>
#include <opencog/xml/StringXMLBufferReader.h>

using namespace LearningServerMessages;

/**
 * Constructor and destructor
 */
StopLearningMessage::~StopLearningMessage(){
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to) : 
  Message(from, to, MessagingSystem::Message::STOP_LEARNING){
	schema.assign("");
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to, 
						   const std::string &msg) :
  Message(from, to, MessagingSystem::Message::STOP_LEARNING){
						   	
	loadPlainTextRepresentation(msg.c_str());
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to,
                           const std::string &schm,  const std::vector<std::string> &argumentsList)
                           throw (opencog::InvalidParamException, std::bad_exception): 
  Message(from, to, MessagingSystem::Message::STOP_LEARNING) { 

    schema.assign(schm);
    setSchemaArguments(argumentsList);
}

/**
 * Public methods
 */
const char * StopLearningMessage::getPlainTextRepresentation(){
	message.assign("");
	
	message.append(schema);
	message.append(END_TOKEN);
	foreach(std::string s, schemaArguments) {
	  message.append(s);
	  message.append(END_TOKEN);	  
	}

	return message.c_str();	
}

void StopLearningMessage::loadPlainTextRepresentation(const char *strMessage) {

	opencog::StringTokenizer stringTokenizer((std::string)strMessage, (std::string)END_TOKEN);

	schema = stringTokenizer.nextToken();

	schemaArguments.clear();
	std::string s = stringTokenizer.nextToken();
	while(!s.empty()) {
	  schemaArguments.push_back(s);
	  s = stringTokenizer.nextToken();
	}
}

/**
 * Getters and setters
 */
void StopLearningMessage::setSchema(const std::string &schm){
	schema.assign(schm);
}

const std::string & StopLearningMessage::getSchema(){
	return schema;
}

const std::vector<std::string> & StopLearningMessage::getSchemaArguments(){
	return schemaArguments;
}

void StopLearningMessage::setSchemaArguments(const std::vector<std::string> &argumentsList)
{
    schemaArguments = argumentsList;
}
