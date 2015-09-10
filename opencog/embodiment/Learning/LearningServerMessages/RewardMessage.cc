/*
 * opencog/embodiment/Learning/LearningServerMessages/RewardMessage.cc
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

#include <stdlib.h>

#include <opencog/util/StringTokenizer.h>
#include <opencog/util/StringManipulator.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>

#include "RewardMessage.h"

using namespace opencog::learningserver::messages;
using namespace opencog::messaging;

int RewardMessage::_rewardMsgType = init();

static Message* rewardFactory(const std::string &from, const std::string &to,
                              int msgType, const std::string &msg)
{
    return new RewardMessage(from, to, msg);
}

int RewardMessage::init()
{
   _rewardMsgType = registerMessageFactory((factory_t) rewardFactory, REWARD);
   return _rewardMsgType;
}

const double RewardMessage::POSITIVE = 1.0;
const double RewardMessage::NEGATIVE = 0.0;

RewardMessage::~RewardMessage()
{
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to) :
        Message(from, to, _rewardMsgType)
{

    schema.assign("");
    candidateSchema.assign("");
    reward = NEGATIVE;
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to,
                             const std::string &msg) :
        Message(from, to, _rewardMsgType)
{

    loadPlainTextRepresentation(msg.c_str());
}

RewardMessage::RewardMessage(const std::string &from, const std::string &to,
                             const std::string &schema,
                             const std::vector<std::string> & schemaArgs,
                             const std::string &candidateSchema,
                             double reward) : Message(from, to, _rewardMsgType)
{
    this->reward = reward;
    this->schema.assign(schema);
    this->candidateSchema.assign(candidateSchema);
    this->schemaArguments = schemaArgs;
}

/**
 * Public methods
 */
const char * RewardMessage::getPlainTextRepresentation()
{
    message.assign("");

    message.append(schema);
    message.append(END_TOKEN);

    for (const std::string& sa : schemaArguments) {
        message.append(sa);
        message.append(END_TOKEN);
    }
    message.append(candidateSchema);
    message.append(END_TOKEN);
    message.append(opencog::toString(reward));

    return message.c_str();
}

void RewardMessage::loadPlainTextRepresentation(const char *strMessage)
{
    opencog::StringTokenizer stringTokenizer((std::string)strMessage,
            (std::string)END_TOKEN);

    schema = stringTokenizer.nextToken();
    candidateSchema = stringTokenizer.nextToken();
    reward = atof(stringTokenizer.nextToken().c_str());
}

/**
 * Atribute methods
 */
void RewardMessage::setSchema(const std::string &schema)
{
    this->schema.assign(schema);
}

const std::string RewardMessage::getSchema()
{
    return schema;
}

void  RewardMessage::setCandidateSchema(const std::string &candidateSchema)
{
    this->candidateSchema.assign(candidateSchema);
}

const std::string RewardMessage::getCandidateSchema()
{
    return candidateSchema;
}

void RewardMessage::setReward(double reward)
{
    this->reward = reward;
}

double RewardMessage::getReward()
{
    return reward;
}
