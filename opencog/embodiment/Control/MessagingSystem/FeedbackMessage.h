/**
 * FeedbackMessage.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 14:19:25 BRT 2007
 */

#ifndef FEEDBACKMESSAGE_H
#define FEEDBACKMESSAGE_H

#include <string>
#include "Message.h"

namespace MessagingSystem {

class FeedbackMessage : public Message {

    private:

        std::string petId;
        std::string feedback;

        // the full message buffer
        std::string buffer;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~FeedbackMessage();
        FeedbackMessage(const std::string &from, const std::string &to);
        FeedbackMessage(const std::string &from, const std::string &to, const std::string &msg);
        FeedbackMessage(const std::string &from, const std::string &to, const std::string &petId, const std::string &msg);

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
        void loadPlainTextRepresentation(const char *strMessage);

        // ***********************************************/
        // Getters and setters

        void setPetId(const std::string &petId);
        void setFeedback(const std::string &msg);
        void setFeedback(const char *msg);
        
        const std::string& getFeedback();
        const std::string& getPetId();


}; // class
}  // namespace

#endif
