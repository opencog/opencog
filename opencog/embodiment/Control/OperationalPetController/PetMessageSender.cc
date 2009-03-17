/** 
 * PetMessageSender.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include "PAIUtils.h"
#include "LearnMessage.h"
#include "LSCmdMessage.h"
#include "RewardMessage.h"
#include "FeedbackMessage.h"
#include "TrySchemaMessage.h"
#include "StopLearningMessage.h"

#include "PetMessageSender.h"

using namespace OperationalPetController;

PetMessageSender::~PetMessageSender(){
}

PetMessageSender::PetMessageSender(MessagingSystem::NetworkElement * ne){
	this->ne = ne;
}

bool PetMessageSender::sendReward(const std::string &schema, const std::vector<std::string> & schemaArguments,
								 const std::string &triedSchema, 
								 const double reward){
								 	
	LearningServerMessages::RewardMessage msg(ne->getID(), ne->parameters.get("LS_ID"), schema, schemaArguments, triedSchema, reward);
	return ne->sendMessage(msg);
}

bool PetMessageSender::sendExemplar(const std::string &schema, const std::vector<std::string> & schemaArguments, 
                                   const std::string &ownerId,
                                   const std::string &avatarId,
								   SpaceServer &spaceServer){
								   	
	LearningServerMessages::LearnMessage msg(ne->getID(), ne->parameters.get("LS_ID"), schema, schemaArguments, ownerId, avatarId, spaceServer);
    MAIN_LOGGER.log(LADSUtil::Logger::DEBUG, "PetMessageSender - sending exemplar message for schema '%s'.", schema.c_str()); 
	return ne->sendMessage(msg);
}

bool PetMessageSender::sendCommand(const std::string &command, 
								  const std::string &schema){

	LearningServerMessages::LSCmdMessage msg(ne->getID(), ne->parameters.get("LS_ID"), command, schema);
	return ne->sendMessage(msg);
}

bool PetMessageSender::sendFeedback(const std::string &petId, const std::string &feedback){
    
    MessagingSystem::FeedbackMessage msg(ne->getID(), ne->parameters.get("PROXY_ID"), 
                                         PerceptionActionInterface::PAIUtils::getExternalId(petId.c_str()), feedback);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs){

    LearningServerMessages::TrySchemaMessage msg(ne->getID(), ne->parameters.get("LS_ID"), schemaName, schemaArgs);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs){
    
    LearningServerMessages::StopLearningMessage msg(ne->getID(), ne->parameters.get("LS_ID"), schemaName, schemaArgs);
    return ne->sendMessage(msg);
}
