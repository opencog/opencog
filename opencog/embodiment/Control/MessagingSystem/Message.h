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

namespace MessagingSystem
{

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

    // Message types (used in factory method)
    static const int STRING = 1;
    static const int LEARN  = 2;
    static const int REWARD = 3;
    static const int SCHEMA = 4;
    static const int LS_CMD = 5;
    static const int ROUTER = 6;
    static const int CANDIDATE_SCHEMA = 7;
    static const int TICK = 8;
    static const int FEEDBACK = 9;
    static const int TRY = 10;
    static const int STOP_LEARNING = 11;

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

    /**
     * Builds Message object of given type
     *
     * @return A new Message of given type
     */
    static Message *factory(const std::string &from, const std::string &to,
                int msgType, const std::string &msg)
            throw (opencog::InvalidParamException, std::bad_exception);

    /**
     * Built a message object of RouterMessage type. This method should be
     * called ONLY by Router related classes.
     *
     * @return A new message
     */
    static Message *routerMessageFactory(const std::string &from,
            const std::string &to, int encapsulateMsgType, const std::string &msg);

    // Setters and getters

    const std::string &getFrom() const;
    void setFrom(const std::string &from);
    const std::string &getTo() const;
    void setTo(const std::string &to);
    int getType() const;
    void setType(int type);

}; // class
}  // namespace

#endif
