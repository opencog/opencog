/*
 * opencog/embodiment/Learning/LearningServer/LSMocky.cc
 *
 * Copyright (C) 2007-2008 Carlos Lopes
 * All Rights Reserved
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
#include "LSMocky.h"
#include "SleepAgent.h"
#include <opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LearnMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/RewardMessage.h>

using namespace LearningServer;
using namespace opencog;
using namespace MessagingSystem;

/**
 * Constructor and destructor
 */
BaseServer* LSMocky::createInstance()
{
    return new LSMocky;
}

LSMocky::LSMocky()
{
}

void LSMocky::init(const std::string &myId, const std::string &ip,
                   int portNumber)
{
    setNetworkElement(new NetworkElement(myId, ip, portNumber));
    registerAgent(SleepAgent::info().id, &sleepAgentFactory);
    SleepAgent* sleepAgent = static_cast<SleepAgent*>(
                                 createAgent(SleepAgent::info().id, &sleepAgentFactory));
    startAgent(sleepAgent);
}

LSMocky::~LSMocky()
{
}


bool LSMocky::processNextMessage(MessagingSystem::Message *msg)
{
    LearningServerMessages::LearnMessage  * lm;
    LearningServerMessages::RewardMessage * rm;
    LearningServerMessages::LSCmdMessage  * cm;

    switch (msg->getType()) {

    case MessagingSystem::Message::LS_CMD:
        cm = (LearningServerMessages::LSCmdMessage *)msg;
        logger().log(opencog::Logger::INFO, "LSMocky - CMD - Command: %s, Pet: %s,  Schema: %s.",
                     cm->getCommand().c_str(),
                     cm->getFrom().c_str(),
                     cm->getSchema().c_str());
        break;

    case MessagingSystem::Message::LEARN:
        lm = (LearningServerMessages::LearnMessage *)msg;
        logger().log(opencog::Logger::INFO, "LSMocky - LEARN - Pet: %s, Learning Schema: %s.",
                     lm->getFrom().c_str(),
                     lm->getSchema().c_str());
        break;

    case MessagingSystem::Message::REWARD:
        rm = (LearningServerMessages::RewardMessage *)msg;
        logger().log(opencog::Logger::INFO, "LSMocky - REWARD - Pet: %s, Tried Schema: %s.",
                     rm->getFrom().c_str(),
                     rm->getCandidateSchema().c_str());

        break;

    default:
        logger().log(opencog::Logger::ERROR, "LSMocky - Unknown message type.");
    }
    return false;
}


