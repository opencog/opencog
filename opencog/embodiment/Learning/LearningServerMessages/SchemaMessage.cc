/**
 * SchemaMessage.cc
 *
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#include <sstream>
#include "SchemaMessage.h"
#include "util/StringTokenizer.h"
#include "PetComboVocabulary.h"

using namespace LearningServerMessages;
using namespace PetCombo;

SchemaMessage::~SchemaMessage(){
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to) :
							 Message(from, to, MessagingSystem::Message::SCHEMA) {
	schema.assign("");
	schemaName.assign("");
	candidateSchemaName.assign("");
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
						     const std::string &msg, int msgType) :
						     Message(from, to, msgType){

	loadPlainTextRepresentation(msg.c_str());
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
		      				 const combo::combo_tree & comboSchema, const std::string &schemaName,
		      				 const std::string &candidateSchemaName) :
		      				 Message(from, to, MessagingSystem::Message::SCHEMA) {

	this->schemaName.assign(schemaName);
	if(candidateSchemaName.size() != 0){
		this->candidateSchemaName.assign(candidateSchemaName);
		setType(CANDIDATE_SCHEMA);
	}

	setSchema(comboSchema);
}

const char * SchemaMessage::getPlainTextRepresentation(){
	message.assign("");

	message.append(schemaName);

	// candidate schema messages has an extra parameter
	if(getType() == CANDIDATE_SCHEMA){
		message.append(END_TOKEN);
		message.append(candidateSchemaName);
	}

	message.append(END_TOKEN);
	message.append(schema);

	return message.c_str();
}

void SchemaMessage::loadPlainTextRepresentation(const char *strMessage)
                        throw (opencog::InvalidParamException, std::bad_exception){

	opencog::StringTokenizer stringTokenizer((std::string)strMessage, (std::string)END_TOKEN);

	schemaName = stringTokenizer.nextToken();
    if(schemaName.empty()){
        throw opencog::InvalidParamException(TRACE_INFO, "Cannot create a SchemaMessage with an empty name");
    }

    if(getType() == CANDIDATE_SCHEMA){

        candidateSchemaName = stringTokenizer.nextToken();
        if(schemaName.empty()){
            throw opencog::InvalidParamException(TRACE_INFO,
                              "Cannot create a CandidateSchemaMessage with an empty candidate name");
        }
	}

	schema = stringTokenizer.nextToken();
    if(schema.empty()){
        throw opencog::InvalidParamException(TRACE_INFO, "Cannot create a SchemaMessage with an empty schema");
    }
}

void SchemaMessage::setSchema(const combo::combo_tree & comboSchema){
	std::stringstream stream;
	stream << comboSchema;

	this->schema = stream.str();
}

const combo::combo_tree SchemaMessage::getComboSchema(){
    using namespace combo;

	combo::combo_tree comboSchema;
	std::stringstream stream(schema);

	stream >> comboSchema;

	return comboSchema;
}

const std::string & SchemaMessage::getSchema(){
	return schema;
}

void SchemaMessage::setSchemaName(const std::string & schemaName){
	this->schemaName.assign(schemaName);
}

const std::string & SchemaMessage::getSchemaName(){
	return schemaName;
}

void SchemaMessage::setCandidateSchemaName(const std::string & candidateSchemaName){
	this->candidateSchemaName.assign(candidateSchemaName);
}

const std::string & SchemaMessage::getCandidateSchemaName(){
	return candidateSchemaName;
}
