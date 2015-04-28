/*
 * opencog/embodiment/Control/MessagingSystem/Message.h
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


#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>
#include <exception>
#include <opencog/util/exceptions.h>

#define END_TOKEN "***"
#define sizeOfToken() strlen(END_TOKEN);

namespace opencog { namespace messaging {

/**
 * Interface supposed to be implemented by classes which actually
 * carry messages to be exchanged between NetworkElements.
 */
class Message
{

private:

    std::string from; //! ID of source NetworkElement
    std::string to;   //! ID of target NetworkElement
    int type; //! Message type (used in factory method to build messages of given types)

public:

    virtual ~Message();

    /**
     * Default constructor which just sets state variables
     */
    Message(const std::string &from, const std::string &to, int type);

    /**
     * Return a const c-style string representation of the message terminated with '\0'.
     *
     * @return A (char *) representation of the message, a c-style string terminated with '\0'
     */
    virtual const char *getPlainTextRepresentation() = 0;

    /**
     * Factory to create a message from a c-style (char *) string terminated with `\0`.
     *
     * @param strMessage (char *) representation of the message to be built.
     */
    virtual void loadPlainTextRepresentation(const char *strMessage) = 0;

    // Setters and getters

    const std::string &getFrom() const;
    void setFrom(const std::string &from);
    const std::string &getTo() const;
    void setTo(const std::string &to);
    int getType() const;
    void setType(int type);

}; // class
} } // namespace opencog::messaging

#endif
