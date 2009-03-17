/** 
 * PetMessageSender.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef PETMESSAGESENDER_H_
#define PETMESSAGESENDER_H_

#include "MessageSender.h"
#include "NetworkElement.h"

namespace OperationalPetController{

class PetMessageSender : public MessageSender {
	
	private:
	
        /**
         * A network element object used to send the messages to LS
         */	
		MessagingSystem::NetworkElement * ne;
		
	public:
	
		/** 
		 * Constructor
		 */
		~PetMessageSender();
		PetMessageSender(MessagingSystem::NetworkElement * ne);
		
		/**
		 * 
		 */
		bool sendReward(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &triedSchema, const double reward);
		
		/**
		 * 
		 */
		bool sendExemplar(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &ownerId, const std::string &avatarId, SpaceServer &spaceServer);
	
		/**
		 * 
		 */
		bool sendCommand(const std::string &command, const std::string &schema);
	
        /**
         *
         */
        bool sendFeedback(const std::string &ownerId, const std::string &feedback);

       /**
         *  
         */
        bool sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs);
	
        /**
         *  
         */
        bool sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs);

        
}; // class
}  // namespace


#endif 

