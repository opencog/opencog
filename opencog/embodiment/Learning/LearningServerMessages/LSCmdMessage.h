/** 
 * LSCmdMessage.h
 * 
 * Author: Carlos Lopes
 * Copyright(c), 2007
 */
#ifndef LSCMDMESSAGE_H_
#define LSCMDMESSAGE_H_

#include "Message.h"

namespace LearningServerMessages {

class LSCmdMessage : public MessagingSystem::Message {

	private:
		std::string schema;
		std::string command;
		
		// the full message to be sent
		std::string message;		
		
	public:
		~LSCmdMessage();
		LSCmdMessage(const std::string &from, const std::string &to);
		LSCmdMessage(const std::string &from, const std::string &to, 
					 const std::string &msg);			
		LSCmdMessage(const std::string &from, const std::string &to, 
				     const std::string &command, const std::string &schema);
		
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
        void loadPlainTextRepresentation(const char *strimessage);
        
        void setCommand(const std::string & command);
        const std::string & getCommand();
        
        void setSchema(const std::string & schema);
        const std::string & getSchema();		

}; // class
}  // namespcae

#endif 
