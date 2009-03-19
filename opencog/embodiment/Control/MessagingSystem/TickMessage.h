/**
 * TickMessage.h
 *
 * $Header$
 *
 * Author: Elvys Borges
 * Creation: Wed Feb 14 14:19:25 BRT 2008
 */

#ifndef TICKMESSAGE_H
#define TICKMESSAGE_H

#include <string>
#include "Message.h"

namespace MessagingSystem
{

class TickMessage : public Message
{

private:

public:

    // ***********************************************/
    // Constructors/destructors

    ~TickMessage();
    TickMessage(const std::string &from, const std::string &to);

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


}; // class
}  // namespace

#endif
