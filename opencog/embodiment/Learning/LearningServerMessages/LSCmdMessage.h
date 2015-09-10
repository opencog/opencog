/*
 * opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Carlos Lopes, Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef LSCMDMESSAGE_H_
#define LSCMDMESSAGE_H_

#include <opencog/embodiment/Control/MessagingSystem/Message.h>

namespace opencog { namespace learningserver { namespace messages {

class LSCmdMessage : public opencog::messaging::Message
{

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

    static int _lscmdMsgType;
private:
    static int init();

}; // class
} } } // namespace opencog::learningserver::messages

#endif
