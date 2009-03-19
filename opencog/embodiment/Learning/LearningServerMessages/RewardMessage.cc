/**
 * RewardMessage.cc
 *
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */

#include "RewardMessage.h"
#include "util/StringTokenizer.h"
#include "util/StringManipulator.h"
#include "stdlib.h"

using namespace LearningServerMessages;

RewardMessage::~RewardMessage(){
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to) :
							 Message(from, to, MessagingSystem::Message::REWARD){

	schema.assign("");
	candidateSchema.assign("");
	reward = NEGATIVE;
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to,
						   const std::string &msg) :
						   Message(from, to, MessagingSystem::Message::REWARD){

	loadPlainTextRepresentation(msg.c_str());
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to,
						     const std::string &schema,
                             const std::vector<std::string> & schemaArgs,
                             const std::string &candidateSchema,
					         double reward) : Message(from, to, MessagingSystem::Message::REWARD){
	this->reward = reward;
	this->schema.assign(schema);
	this->candidateSchema.assign(candidateSchema);
    this->schemaArguments = schemaArgs;
}

/**
 * Public methods
 */
const char * RewardMessage::getPlainTextRepresentation(){
	message.assign("");

	message.append(schema);
	message.append(END_TOKEN);

    for( std::vector<std::string>::iterator it = schemaArguments.begin(); it != schemaArguments.end(); it++ )
    {
    	message.append(*it);
	    message.append(END_TOKEN);
    }
	message.append(candidateSchema);
	message.append(END_TOKEN);
	message.append(opencog::toString(reward));

	return message.c_str();
}

void RewardMessage::loadPlainTextRepresentation(const char *strMessage) {
	opencog::StringTokenizer stringTokenizer((std::string)strMessage,
										  (std::string)END_TOKEN);

	schema = stringTokenizer.nextToken();
	candidateSchema = stringTokenizer.nextToken();
	reward = atof(stringTokenizer.nextToken().c_str());
}

/**
 * Atribute methods
 */
void RewardMessage::setSchema(const std::string &schema){
	this->schema.assign(schema);
}

const std::string RewardMessage::getSchema(){
	return schema;
}

void  RewardMessage::setCandidateSchema(const std::string &candidateSchema){
	this->candidateSchema.assign(candidateSchema);
}

const std::string RewardMessage::getCandidateSchema(){
	return candidateSchema;
}

void RewardMessage::setReward(double reward){
	this->reward = reward;
}

double RewardMessage::getReward(){
	return reward;
}
