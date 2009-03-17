/** 
 * MessageSender.h
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#ifndef MESSAGESSENDER_H_
#define MESSAGESSENDER_H_

#include <string>
#include <opencog/atomspace/HandleEntry.h>

#include "SpaceServer.h"

namespace OperationalPetController{

class MessageSender{
	
	public:
		virtual ~MessageSender() {}
	
		/**
		 *  
		 * @return a boolean value that indicates the success (true) or failure (false) of this sending operation.
		 */ 	
		virtual bool sendReward(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &triedSchema, const double reward) = 0;
		
		/**
		 * 
		 */
		virtual bool sendExemplar(const std::string &schema, const std::vector<std::string> & schemaArguments, const std::string &ownerId, const std::string &avatarId, SpaceServer& spaceServer) = 0;
	
		/**
		 * 
		 */
		virtual bool sendCommand(const std::string &command, const std::string &schema) = 0;

        /**
         *
         */ 
        virtual bool sendFeedback(const std::string &ownerId, const std::string &feedback) = 0;

        /**
         *  
         */
        virtual bool sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs) = 0;
	
        /**
         *  
         */
        virtual bool sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs) = 0;

}; // class
}  // namespace
#endif 
