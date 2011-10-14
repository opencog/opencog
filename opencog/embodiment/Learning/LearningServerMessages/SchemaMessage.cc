/*
 * opencog/embodiment/Learning/LearningServerMessages/SchemaMessage.cc
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

#include <sstream>
#include "SchemaMessage.h"
#include <opencog/util/StringTokenizer.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

using namespace opencog::learningserver::messages;
using namespace PetCombo;

SchemaMessage::~SchemaMessage()
{
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to) :
        Message(from, to, opencog::messaging::Message::SCHEMA)
{
    schema.assign("");
    schemaName.assign("");
    candidateSchemaName.assign("");
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
                             const std::string &msg, int msgType) :
        Message(from, to, msgType)
{

    loadPlainTextRepresentation(msg.c_str());
}

SchemaMessage::SchemaMessage(const std::string &from, const std::string &to,
                             const opencog::combo::combo_tree & comboSchema, const std::string &schemaName,
                             const std::string &candidateSchemaName) :
        Message(from, to, opencog::messaging::Message::SCHEMA)
{

    this->schemaName.assign(schemaName);
    if (candidateSchemaName.size() != 0) {
        this->candidateSchemaName.assign(candidateSchemaName);
        setType(CANDIDATE_SCHEMA);
    }

    setSchema(comboSchema);
}

const char * SchemaMessage::getPlainTextRepresentation()
{
    message.assign("");

    message.append(schemaName);

    // candidate schema messages has an extra parameter
    if (getType() == CANDIDATE_SCHEMA) {
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

    if (getType() == CANDIDATE_SCHEMA) {

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
