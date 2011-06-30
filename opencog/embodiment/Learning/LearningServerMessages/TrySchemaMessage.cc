/*
 * opencog/embodiment/Learning/LearningServerMessages/TrySchemaMessage.cc
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


#include <vector>
#include "TrySchemaMessage.h"

#include <opencog/atomspace/SpaceServer.h>
#include <opencog/util/StringTokenizer.h>

#include <opencog/persist/xml/NMXmlParser.h>
#include <opencog/persist/xml/NMXmlExporter.h>
#include <opencog/persist/xml/StringXMLBufferReader.h>

using namespace LearningServerMessages;

/**
 * Constructor and destructor
 */
TrySchemaMessage::~TrySchemaMessage()
{
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to) :
        Message(from, to, opencog::messaging::Message::TRY)
{
    schema.assign("");
    schemaArguments.clear();
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to,
                                   const std::string &msg) :
        Message(from, to, opencog::messaging::Message::TRY)
{

    loadPlainTextRepresentation(msg.c_str());
}

TrySchemaMessage::TrySchemaMessage(const std::string &from, const std::string &to,
                                   const std::string &schm,  const std::vector<std::string> &argumentsList)
throw (opencog::InvalidParamException, std::bad_exception):
        Message(from, to, opencog::messaging::Message::TRY)
{

    schema.assign(schm);
    setSchemaArguments(argumentsList);
}

/**
 * Public methods
 */
const char * TrySchemaMessage::getPlainTextRepresentation()
{
    message.assign("");

    message.append(schema);
    message.append(END_TOKEN);
    foreach(std::string s, schemaArguments) {
        message.append(s);
        message.append(END_TOKEN);
    }

    return message.c_str();
}

void TrySchemaMessage::loadPlainTextRepresentation(const char *strMessage)
{

    opencog::StringTokenizer stringTokenizer((std::string)strMessage, (std::string)END_TOKEN);

    schema = stringTokenizer.nextToken();

    schemaArguments.clear();
    std::string s = stringTokenizer.nextToken();
    while (!s.empty()) {
        schemaArguments.push_back(s);
        s = stringTokenizer.nextToken();
    }
}

/**
 * Getters and setters
 */
void TrySchemaMessage::setSchema(const std::string &schm)
{
    schema.assign(schm);
}

const std::string & TrySchemaMessage::getSchema()
{
    return schema;
}

const std::vector<std::string> & TrySchemaMessage::getSchemaArguments()
{
    return schemaArguments;
}

void TrySchemaMessage::setSchemaArguments(const std::vector<std::string> &argumentsList)
{
    schemaArguments = argumentsList;
}
