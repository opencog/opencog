/** 
 * StopLearningMessage.h
 * 
 * Author: Erickson Nascimento
 * Copyright(c), 2008
 */

#ifndef STOPLEARNINGMESSAGE_H_
#define STOPLEARNINGMESSAGE_H_

#include "Message.h"
#include <opencog/atomspace/AtomSpace.h>

namespace LearningServerMessages {

class StopLearningMessage : public MessagingSystem::Message {
	
	private:
		std::string schema;
        std::vector<std::string> schemaArguments;
			
		// the full message to be sent
		std::string message;
		
	public:
		
		~StopLearningMessage();
		StopLearningMessage(const std::string &from, const std::string &to);
		StopLearningMessage(const std::string &from, const std::string &to, 
					 const std::string &msg);	
		StopLearningMessage(const std::string &from, const std::string &to, 
				     const std::string &schema, const std::vector<std::string> &argumentsList)
                     throw (opencog::InvalidParamException, std::bad_exception);
		
        /**
         * Return A (char *) representation of the message, a c-style string terminated with '\0'.
         * Returned string is a const pointer hence it shaw not be modified and there is no need to
         * free/delete it.
         *
         * @return A (char *) representation of the message, a c-style string terminated with '\0'
         */
        const char *getPlainTextRepresentation();

        /**
         * Factory a message using a c-style (char *) string terminated with `\0`.
         *
         * @param strMessage (char *) representation of the message to be built.
         */
        void loadPlainTextRepresentation(const char *strMessage);
        
        /**
         * Schema getter and setter
         */
       	void setSchema(const std::string &schema); 
        const std::string & getSchema();

        /**
         * Schema arguments get and set methods.
         */
        const std::vector<std::string> &getSchemaArguments();
        void setSchemaArguments(const std::vector<std::string> &argumentsList);

}; // class
}  // namespace
#endif 
