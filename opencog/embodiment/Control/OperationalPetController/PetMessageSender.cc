/*
 * opencog/embodiment/Control/MessagingSystem/PetMessageSender.cc
 *
 * Copyright (C) 2007-2008 Carlos Lopes
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
    logger().log(opencog::Logger::DEBUG, "PetMessageSender - sending exemplar message for schema '%s'.", schema.c_str()); 
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
