/*
 * opencog/embodiment/Learning/LearningServerMessages/StopLearningMessage.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Erickson Nascimento
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


#ifndef STOPLEARNINGMESSAGE_H_
#define STOPLEARNINGMESSAGE_H_

#include <opencog/embodiment/Control/MessagingSystem/Message.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog { namespace learningserver { namespace messages {

class StopLearningMessage : public opencog::messaging::Message
{

private:
    std::string schema;
    std::vector<std::string> schemaArguments;

    // the full message to be sent
    std::string message;

public:

    ~StopLearningMessage();
    StopLearningMessage(const std::string &from, const std::string &to);
    StopLearningMessage(const std::string &from, const std::string &to,
                        const std::string &msg);
    StopLearningMessage(const std::string &from, const std::string &to,
                        const std::string &schema, const std::vector<std::string> &argumentsList)
    throw (opencog::InvalidParamException, std::bad_exception);

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

    /**
     * Schema getter and setter
     */
    void setSchema(const std::string &schema);
    const std::string & getSchema();

    /**
     * Schema arguments get and set methods.
     */
    const std::vector<std::string> &getSchemaArguments();
    void setSchemaArguments(const std::vector<std::string> &argumentsList);

    static int _stopMsgType;
private:
    static int init();

}; // class
} } } // namespace opencog::learningserver::messages
#endif
