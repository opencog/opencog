/*
 * opencog/embodiment/Control/MessagingSystem/EmbodimentCogServer.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


#include "EmbodimentCogServer.h"

#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>

using namespace opencog;
using namespace MessagingSystem;

EmbodimentCogServer::EmbodimentCogServer()
{
    logger().info("[EmbodimentCogServer] constructor");
}

EmbodimentCogServer::~EmbodimentCogServer()
{
    logger().info("[EmbodimentCogServer] destructor");
}

void EmbodimentCogServer::setNetworkElement(NetworkElement* _ne)
{
    ne = _ne;
    externalTickMode = config().get_bool("EXTERNAL_TICK_MODE");
    unreadMessagesCheckInterval = opencog::config().get_int("UNREAD_MESSAGES_CHECK_INTERVAL");
    unreadMessagesRetrievalLimit = opencog::config().get_int("UNREAD_MESSAGES_RETRIEVAL_LIMIT");
    maxMessageQueueSize = opencog::config().get_int("MAX_MESSAGE_QUEUE_SIZE"); 
}

NetworkElement& EmbodimentCogServer::getNetworkElement(void)
{
    return *ne;
}

bool EmbodimentCogServer::customLoopRun(void)
{
    unsigned int msgQueueSize = ne->getIncomingQueueSize();

    if (msgQueueSize > 0) {
        logger().debug("EmbodimentCogServer(%s) - messageCentral size: %d [cycle = %d]",
                       ne->getID().c_str(), 
                       msgQueueSize,
                       this->getCycleCount()
                      );
    }

    bool runAgents = false;

    if (!externalTickMode) {
        // Retrieve new messages from router, if any
        if ((cycleCount % unreadMessagesCheckInterval) == 0) {
            ne->retrieveMessages(unreadMessagesRetrievalLimit);
        }

        // Process all messages in queue
        std::vector<Message*> messages;
        while (!ne->isIncomingQueueEmpty() ) {
            Message *message = ne->popIncomingQueue();
            messages.push_back(message);
        }
        for (unsigned int i = 0; i < messages.size(); i++) {
            Message *message = messages[i];
            if (message->getType() != Message::TICK) {
                // If processNextMessage returns false then we stop the
                // CogServer
                this->running = !processNextMessage(message);
                delete(message);
                if (!this->running) return false;
            } else {
                logger().warn("EmbodimentCogServer - got a TICK, but not using external tick mode.");
                delete(message);
            }
        }
        return true;
    } else {
        ne->retrieveMessages(unreadMessagesRetrievalLimit);

        // Get all messages in the incoming queue (or until a TICK message is received)
        std::vector<Message*> messages;
        while (!ne->isIncomingQueueEmpty()) {
            Message *message = ne->popIncomingQueue();
            messages.push_back(message);
            //if (message->getType() == Message::TICK) break;
        }
        for (unsigned int i = 0; i < messages.size(); i++) {
            Message *message = messages[i];
            //check type of message
            if (message->getType() == Message::TICK) {
                logger().debug("EmbodimentCogServer - No.%d message is a TICK, %d messages still unprocessed. Will process them then run MindAgents [cycle = %d]",
                               i+1, msgQueueSize-i-1, this->getCycleCount()
                              );
                delete(message);
                runAgents = true;
            } else {
                // If processNextMessage returns true then we stop the
                // CogServer. It only returns true when the spawner
                // sends a SAVE_AND_EXIT message to this OAC.
                this->running = !processNextMessage(message);
                delete(message);

                if (!running) {
                    logger().debug("EmbodimentCogServer - Shutting down. Failed to process No.%d message, %d messages left unprocessed [cycle = %d]",
                                   i+1, msgQueueSize-i-1, this->getCycleCount()
                                  );

                    return false;
                }
            }
        }
    }

    //sleep a bit to not keep cpu busy and do not keep requesting for unread messages
    //usleep(50000);

    return runAgents;
}

bool EmbodimentCogServer::sendMessage(Message &msg)
{
    return ne->sendMessage(msg);
}

bool EmbodimentCogServer::sendCommandToRouter(const std::string &cmd)
{
    return ne->sendCommandToRouter(cmd);
}

const std::string& EmbodimentCogServer::getID()
{
    return ne->getID();
}

int EmbodimentCogServer::getPortNumber()
{
    return ne->getPortNumber();
}

bool EmbodimentCogServer::isElementAvailable(const std::string& id)
{
    return ne->isElementAvailable(id);
}

void EmbodimentCogServer::logoutFromRouter()
{
    return ne->logoutFromRouter();
}
