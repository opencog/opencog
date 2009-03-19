/** 
 * TrySchemaMessage.cc
 * 
 * Author: Erickson Nascimento
 * Copyright(c), 2008
 */

#include <vector>
#include "TrySchemaMessage.h"

#include "SpaceServer.h"
#include "util/StringTokenizer.h"

#include <opencog/xml/NMXmlParser.h>
#include <opencog/xml/NMXmlExporter.h>
#include <opencog/xml/StringXMLBufferReader.h>

using namespace LearningServerMessages;

/**
 * Constructor and destructor
 */
TrySchemaMessage::~TrySchemaMessage(){
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to) : 
						   Message(from, to, MessagingSystem::Message::TRY){
	schema.assign("");
    schemaArguments.clear();
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to, 
						   const std::string &msg) :
						   Message(from, to, MessagingSystem::Message::TRY){
						   	
	loadPlainTextRepresentation(msg.c_str());
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to,
                           const std::string &schm,  const std::vector<std::string> &argumentsList)
                           throw (opencog::InvalidParamException, std::bad_exception): 
                           Message(from, to, MessagingSystem::Message::TRY) { 

    schema.assign(schm);
    setSchemaArguments(argumentsList);
}

/**
 * Public methods
 */
const char * TrySchemaMessage::getPlainTextRepresentation(){
	message.assign("");
	
	message.append(schema);
	message.append(END_TOKEN);
	foreach(std::string s, schemaArguments) {
	  message.append(s);
	  message.append(END_TOKEN);	  
	}

	return message.c_str();	
}

void TrySchemaMessage::loadPlainTextRepresentation(const char *strMessage) {

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
void TrySchemaMessage::setSchema(const std::string &schm){
	schema.assign(schm);
}

const std::string & TrySchemaMessage::getSchema(){
	return schema;
}

const std::vector<std::string> & TrySchemaMessage::getSchemaArguments(){
	return schemaArguments;
}

void TrySchemaMessage::setSchemaArguments(const std::vector<std::string> &argumentsList)
{
    schemaArguments = argumentsList;
}
