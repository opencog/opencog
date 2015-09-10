/*
 * opencog/embodiment/Learning/LearningServer/LSMocky.cc
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


#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LSCmdMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/LearnMessage.h>
#include <opencog/embodiment/Learning/LearningServerMessages/RewardMessage.h>
#include "LSMocky.h"
#include "SleepAgent.h"

using namespace opencog::learningserver;
using namespace opencog;
using namespace opencog::messaging;

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
    SleepAgentPtr sleepAgent = createAgent<SleepAgent>();
    startAgent(sleepAgent);
}

LSMocky::~LSMocky()
{
}


bool LSMocky::processNextMessage(opencog::messaging::Message *msg)
{
    learningserver::messages::LSCmdMessage  * cm;
    cm = dynamic_cast<learningserver::messages::LSCmdMessage*>(msg);
    if (cm) {
        logger().info("LSMocky - CMD - Command: %s, Pet: %s,  Schema: %s.",
                     cm->getCommand().c_str(),
                     cm->getFrom().c_str(),
                     cm->getSchema().c_str());
        return false;
    }

    learningserver::messages::LearnMessage  * lm;
    lm = dynamic_cast<learningserver::messages::LearnMessage*>(msg);
    if (lm) {
        logger().info("LSMocky - LEARN - Pet: %s, Learning Schema: %s.",
                     lm->getFrom().c_str(),
                     lm->getSchema().c_str());
        return false;
    }

    learningserver::messages::RewardMessage * rm;
    rm = dynamic_cast<learningserver::messages::RewardMessage*> (msg);
    if (rm) {
        logger().info("LSMocky - REWARD - Pet: %s, Tried Schema: %s.",
                     rm->getFrom().c_str(),
                     rm->getCandidateSchema().c_str());

        return false;
    }

    logger().error("LSMocky - Unknown message type.");
    return false;
}


