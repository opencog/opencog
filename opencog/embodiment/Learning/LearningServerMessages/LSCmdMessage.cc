/*
 * opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.cc
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

#include "LSCmdMessage.h"
#include <opencog/util/StringTokenizer.h>

using namespace opencog::learningserver::messages;

/**
 * Constructor and destructor
 */
LSCmdMessage::~LSCmdMessage()
{
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to) :
        Message(from, to, opencog::messaging::Message::LS_CMD)
{
    schema.assign("");
    command.assign("");
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to,
                           const std::string &msg) :
        Message(from, to, opencog::messaging::Message::LS_CMD)
{

    loadPlainTextRepresentation(msg.c_str());
}

LSCmdMessage::LSCmdMessage(const std::string &from, const std::string &to,
                           const std::string &command, const std::string &schema) :
        Message(from, to, opencog::messaging::Message::LS_CMD)
{

    this->schema.assign(schema);
    this->command.assign(command);
}

/**
 * Public methods
 */
const char * LSCmdMessage::getPlainTextRepresentation()
{
    message.assign("");

    message.append(command);
    message.append(END_TOKEN);
    message.append(schema);

    return message.c_str();
}

void LSCmdMessage::loadPlainTextRepresentation(const char *strMessage)
{
    opencog::StringTokenizer stringTokenizer((std::string)strMessage,
            (std::string)END_TOKEN);

    command = stringTokenizer.nextToken();
    schema = stringTokenizer.nextToken();
}

void LSCmdMessage::setCommand(const std::string & command)
{
    this->command.assign(command);
}

const std::string & LSCmdMessage::getCommand()
{
    return command;
}

void LSCmdMessage::setSchema(const std::string & schema)
{
    this->schema.assign(schema);
}

const std::string & LSCmdMessage::getSchema()
{
    return schema;
}
