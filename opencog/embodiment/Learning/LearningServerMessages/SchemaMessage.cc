/*
 * opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.cc
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

#include <sstream>
#include <opencog/util/StringTokenizer.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include "SchemaMessage.h"

using namespace opencog::messaging;
using namespace opencog::learningserver::messages;
using namespace AvatarCombo;

int SchemaMessage::_schemaMsgType = -1;
int SchemaMessage::_schemaCandMsgType = init();

static Message* schemaFactory(const std::string &from, const std::string &to,
                              int msgType, const std::string &msg)
{
    return new SchemaMessage(from, to, msg, msgType);
}

int SchemaMessage::init()
{
   _schemaMsgType = registerMessageFactory((factory_t) schemaFactory, SCHEMA);
   _schemaCandMsgType = registerMessageFactory((factory_t) schemaFactory, CANDIDATE_SCHEMA);
   return _schemaCandMsgType;
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to) :
        Message(from, to, _schemaMsgType)
{
    schema.assign("");
    schemaName.assign("");
    candidateSchemaName.assign("");
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
                             const std::string &msg, int msgType) :
        Message(from, to, msgType)
{
    if (-1 == msgType) setType(_schemaMsgType);
    loadPlainTextRepresentation(msg.c_str());
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
                             const opencog::combo::combo_tree & comboSchema, const std::string &schemaName,
                             const std::string &candidateSchemaName) :
        Message(from, to, _schemaMsgType)
{
    this->schemaName.assign(schemaName);
    if (candidateSchemaName.size() != 0) {
        this->candidateSchemaName.assign(candidateSchemaName);
        setType(_schemaCandMsgType);
    }

    setSchema(comboSchema);
}

SchemaMessage::~SchemaMessage()
{
}

const char * SchemaMessage::getPlainTextRepresentation()
{
    message.assign("");

    message.append(schemaName);

    // candidate schema messages has an extra parameter
    if (getType() == _schemaCandMsgType) {
        message.append(END_TOKEN);
        message.append(candidateSchemaName);
    }

    message.append(END_TOKEN);
    message.append(schema);

    return message.c_str();
}

void SchemaMessage::loadPlainTextRepresentation(const char *strMessage)
    throw (opencog::InvalidParamException, std::bad_exception)
{

    opencog::StringTokenizer stringTokenizer((std::string)strMessage, (std::string)END_TOKEN);

    schemaName = stringTokenizer.nextToken();
    if (schemaName.empty()) {
        throw opencog::InvalidParamException(TRACE_INFO, "Cannot create a SchemaMessage with an empty name");
    }

    if (getType() == _schemaCandMsgType) {

        candidateSchemaName = stringTokenizer.nextToken();
        if (schemaName.empty()) {
            throw opencog::InvalidParamException(TRACE_INFO,
                                                 "Cannot create a CandidateSchemaMessage with an empty candidate name");
        }
    }

    schema = stringTokenizer.nextToken();
    if (schema.empty()) {
        throw opencog::InvalidParamException(TRACE_INFO, "Cannot create a SchemaMessage with an empty schema");
    }
}

void SchemaMessage::setSchema(const opencog::combo::combo_tree & comboSchema)
{
    std::stringstream stream;
    stream << comboSchema;

    this->schema = stream.str();
}

const opencog::combo::combo_tree SchemaMessage::getComboSchema()
{
    using namespace opencog::combo;

    combo_tree comboSchema;
    std::stringstream stream(schema);

    stream >> comboSchema;

    return comboSchema;
}

const std::string & SchemaMessage::getSchema()
{
    return schema;
}

void SchemaMessage::setSchemaName(const std::string & schemaName)
{
    this->schemaName.assign(schemaName);
}

const std::string & SchemaMessage::getSchemaName()
{
    return schemaName;
}

void SchemaMessage::setCandidateSchemaName(const std::string & candidateSchemaName)
{
    this->candidateSchemaName.assign(candidateSchemaName);
}

const std::string & SchemaMessage::getCandidateSchemaName()
{
    return candidateSchemaName;
}
