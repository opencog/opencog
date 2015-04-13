/*
 * opencog/embodiment/Control/MessagingSystem/FeedbackMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef FEEDBACKMESSAGE_H
#define FEEDBACKMESSAGE_H

#include <string>
#include "Message.h"

namespace opencog { namespace messaging {

class FeedbackMessage : public Message
{

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

} } // namespace opencog::messaging

#endif
