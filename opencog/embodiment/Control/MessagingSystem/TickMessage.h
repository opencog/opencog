/*
 * opencog/embodiment/Control/MessagingSystem/TickMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Elvys Borges
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


#ifndef TICKMESSAGE_H
#define TICKMESSAGE_H

#include <string>
#include "Message.h"

namespace opencog { namespace messaging {

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
     * Get a (char *) representation of the message, a c-style string
     * terminated with '\0'. Returned string is a const pointer hence it shall
     * not be modified and there is no need to free/delete it.
     *
     * @return A (char *) representation of the message, a c-style string
     * terminated with '\0'
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
} } // namespace opencog::messaging

#endif
