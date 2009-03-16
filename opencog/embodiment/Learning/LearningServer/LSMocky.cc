/** 
 * LSMocky.cc
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#include "LSMocky.h"
#include "SleepTask.h"
#include "LSCmdMessage.h"
#include "LearnMessage.h"
#include "RewardMessage.h"

using namespace LearningServer;

/**
 * Constructor and destructor
 */
LSMocky::LSMocky(const std::string &myId, const std::string &ip, 
       int portNumber, Control::SystemParameters & parameters)
  : NetworkElement(parameters, myId, ip, portNumber) {
}

LSMocky::~LSMocky(){
}

/**
 * Inherated functions from NetworkElement
 */
void LSMocky::setUp(){
    plugInIdleTask(new SleepTask(), 1);
}

bool LSMocky::processNextMessage(MessagingSystem::Message *msg){
	LearningServerMessages::LearnMessage  * lm;
	LearningServerMessages::RewardMessage * rm;
	LearningServerMessages::LSCmdMessage  * cm;
	
	switch(msg->getType()){
		
		case MessagingSystem::Message::LS_CMD:
			cm = (LearningServerMessages::LSCmdMessage *)msg;
            MAIN_LOGGER.log(opencog::Logger::INFO, "LSMocky - CMD - Command: %s, Pet: %s,  Schema: %s.",
                            cm->getCommand().c_str(), 
                            cm->getFrom().c_str(),
                            cm->getSchema().c_str());
		 	break;
		 	
		case MessagingSystem::Message::LEARN:
			lm = (LearningServerMessages::LearnMessage *)msg;
	         MAIN_LOGGER.log(opencog::Logger::INFO, "LSMocky - LEARN - Pet: %s, Learning Schema: %s.",
                         lm->getFrom().c_str(), 
                         lm->getSchema().c_str());
             break;
			
		case MessagingSystem::Message::REWARD:
			rm = (LearningServerMessages::RewardMessage *)msg;
	        MAIN_LOGGER.log(opencog::Logger::INFO, "LSMocky - REWARD - Pet: %s, Tried Schema: %s.",
                         rm->getFrom().c_str(), 
                         rm->getCandidateSchema().c_str());
			
			break;
		
		default: 
			MAIN_LOGGER.log(opencog::Logger::ERROR, "LSMocky - Unknown message type.");
	}
	return false;
}


