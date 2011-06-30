/*
 * opencog/embodiment/Control/MessagingSystem/RouterMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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


#ifndef ROUTERMESSAGE_H
#define ROUTERMESSAGE_H

#include <string>
#include "Message.h"

namespace opencog { namespace messaging {

/**
 * The router message encapsulates all messages to avoid checking messages bad
 * parameters usage in router component. This should be done directly in a
 * NetworkElement object.
 */
class RouterMessage : public Message
{

private:

    int encapsulateType;
    std::string message;

public:

    ~RouterMessage();

    /**
     * Create a message
     *
     * @param from Network Id for the sender of the message
     * @param to Network Id for the receiver of the message
     * @param encapsulateType The type of the message that is going to be
     *                        encapsulated in a Router Message.
     */
    RouterMessage(const std::string &from, const std::string &to, int encapsulateType);

    /**
     * Create a message
     *
     * @param from Network Id for the sender of the message
     * @param to Network Id for the receiver of the message
     * @param encapsulateType The type of the message that is going to be
     *                        encapsulated in a Router Message.
     * @param msg The message itself.
     */
    RouterMessage(const std::string &from, const std::string &to,
                  int encapsulateType, const std::string& msg);

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
    void loadPlainTextRepresentation(const char *message);

    // ***********************************************/
    // Getters and setters

    void  setMessage(const std::string& msg);
    const std::string& getMessage();

    void setEncapsulateType(int encapsulateType);
    int  getEncapsulateType();

}; // class
} } // namespace opencog::messaging

#endif
