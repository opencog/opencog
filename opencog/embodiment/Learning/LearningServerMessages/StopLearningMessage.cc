/*
 * opencog/embodiment/Learning/LearningServerMessages/StopLearningMessage.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * Copyright (C) 2012 Linas Vepstas
 * All Rights Reserved
 * Author(s): Erickson Nascimento, Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/util/foreach.h>
#include <opencog/util/StringTokenizer.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>

#include "StopLearningMessage.h"

using namespace opencog::learningserver::messages;
using namespace opencog::messaging;

int StopLearningMessage::_stopMsgType = init();

static Message* stopFactory(const std::string &from, const std::string &to,
                              int msgType, const std::string &msg)
{
    return new StopLearningMessage(from, to, msg);
}

int StopLearningMessage::init()
{
   _stopMsgType = registerMessageFactory((factory_t) stopFactory, STOP_LEARNING);
   return _stopMsgType;
}


/**
 * Constructor and destructor
 */
StopLearningMessage::~StopLearningMessage()
{
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to) :
        Message(from, to, _stopMsgType)
{
    schema.assign("");
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to,
        const std::string &msg) :
        Message(from, to, _stopMsgType)
{

    loadPlainTextRepresentation(msg.c_str());
}

StopLearningMessage::StopLearningMessage(const std::string &from, const std::string &to,
        const std::string &schm,  const std::vector<std::string> &argumentsList)
throw (opencog::InvalidParamException, std::bad_exception):
        Message(from, to, _stopMsgType)
{

    schema.assign(schm);
    setSchemaArguments(argumentsList);
}

/**
 * Public methods
 */
const char * StopLearningMessage::getPlainTextRepresentation()
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

void StopLearningMessage::loadPlainTextRepresentation(const char *strMessage)
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
void StopLearningMessage::setSchema(const std::string &schm)
{
    schema.assign(schm);
}

const std::string & StopLearningMessage::getSchema()
{
    return schema;
}

const std::vector<std::string> & StopLearningMessage::getSchemaArguments()
{
    return schemaArguments;
}

void StopLearningMessage::setSchemaArguments(const std::vector<std::string> &argumentsList)
{
    schemaArguments = argumentsList;
}
