/*
 * opencog/embodiment/Control/OperationalAvatarController/PetMessageSender.cc
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

#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LearnMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/RewardMessage.h>
#include <opencog/embodiment/Control/MessagingSystem/FeedbackMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/TrySchemaMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/StopLearningMessage.h>
#include <opencog/util/Config.h>

#include "PetMessageSender.h"

using namespace opencog::oac;
using namespace opencog;

PetMessageSender::~PetMessageSender()
{
}

PetMessageSender::PetMessageSender(MessagingSystem::NetworkElement * ne)
{
    this->ne = ne;
}

bool PetMessageSender::sendReward(const std::string &schema, const std::vector<std::string> & schemaArguments,
                                  const std::string &triedSchema,
                                  const double reward)
{

    LearningServerMessages::RewardMessage msg(ne->getID(), config().get("LS_ID"), schema, schemaArguments, triedSchema, reward);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendExemplar(const std::string &schema, const std::vector<std::string> & schemaArguments,
                                    const std::string &ownerId,
                                    const std::string &avatarId,
                                    AtomSpace &atomSpace)
{

    LearningServerMessages::LearnMessage msg(ne->getID(), config().get("LS_ID"), schema, schemaArguments, ownerId, avatarId, atomSpace);
    logger().debug("PetMessageSender - sending exemplar message for schema '%s'.", schema.c_str());
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendCommand(const std::string &command,
                                   const std::string &schema)
{

    LearningServerMessages::LSCmdMessage msg(ne->getID(), config().get("LS_ID"), command, schema);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendFeedback(const std::string &petId, const std::string &feedback)
{

    MessagingSystem::FeedbackMessage msg(ne->getID(), config().get("PROXY_ID"),
                                         opencog::pai::PAIUtils::getExternalId(petId.c_str()), feedback);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendTrySchema(const std::string &schemaName, const std::vector<std::string> & schemaArgs)
{

    LearningServerMessages::TrySchemaMessage msg(ne->getID(), config().get("LS_ID"), schemaName, schemaArgs);
    return ne->sendMessage(msg);
}

bool PetMessageSender::sendStopLearning(const std::string &schemaName, const std::vector<std::string> & schemaArgs)
{

    LearningServerMessages::StopLearningMessage msg(ne->getID(), config().get("LS_ID"), schemaName, schemaArgs);
    return ne->sendMessage(msg);
}
