/**
 * StringMessage.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 14:19:25 BRT 2007
 */

#ifndef STRINGMESSAGE_H
#define STRINGMESSAGE_H

#include <string>
#include "Message.h"

namespace MessagingSystem {

class StringMessage : public Message {

    private:

        std::string message;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~StringMessage();
        StringMessage(const std::string &from, const std::string &to);
        StringMessage(const std::string &from, const std::string &to, const std::string &msg);
        StringMessage(const std::string &from, const std::string &to, const char *msg);

        // ***********************************************/
        // Inherited from message

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

        // ***********************************************/
        // Getters and setters

        void setMessage(const std::string &msg);
        void setMessage(const char *msg);
        const std::string& getMessage();


}; // class
}  // namespace

#endif
