/*
 * opencog/embodiment/Control/MessagingSystem/MemoryMessageCentral.cc
 *
 * Copyright (C) 2010-2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
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

#include "MemoryMessageCentral.h"

using namespace opencog::messaging;

MemoryMessageCentral::~MemoryMessageCentral()
{
    std::map<std::string, std::queue<Message *> *>::iterator map_itr = messageQueue.begin();
    while (map_itr != messageQueue.end()) {
        // TODO: surely we can just removeQueue(strId) here?
        std::string str_id = map_itr->first;
        while (!this->isQueueEmpty(str_id)) {
            Message *msg = this->pop(str_id);
            delete msg;
        }
        delete map_itr->second;
        map_itr++;
    }

}

MemoryMessageCentral::MemoryMessageCentral() : MessageCentral()
{


}

void MemoryMessageCentral::createQueue(const std::string id, const bool reset)
{

    if (reset == false) {
        //test if id already exists in map
        if (!this->existsQueue(id)) {
            std::queue<Message *> *newQueue = new std::queue<Message *>;
            this->lockQueue();
            messageQueue[id] = newQueue;
            this->unlockQueue();
        }
    } else {
        //I need delete all objects in the queue that already exists
        std::queue<Message *> *newQueue = new std::queue<Message *>;
        this->lockQueue();
        messageQueue[id] = newQueue;
        this->unlockQueue();
    }

}

void MemoryMessageCentral::clearQueue(const std::string id)
{

    if (this->existsQueue(id)) {

        // remove all messages
        while (!this->isQueueEmpty(id)) {
            Message * message = this->pop(id);
            delete message;
        }
    }
}


void MemoryMessageCentral::removeQueue(const std::string id)
{

    if (this->existsQueue(id)) {

        clearQueue(id);

        // remove the queue itself
        this->lockQueue();
        std::queue<Message *> *mQueue = messageQueue[id];
        messageQueue.erase(id);
        delete mQueue;
        this->unlockQueue();
    }
}

unsigned int MemoryMessageCentral::queueSize(const std::string id) const
{

    unsigned int value = 0;

    if (this->existsQueue(id)) {
        this->lockQueue();
        const std::queue<Message *> *queue = messageQueue.find(id)->second;
        value = queue->size();
        this->unlockQueue();
    }

    return value;
}

bool MemoryMessageCentral::isQueueEmpty(const std::string id) const
{

    bool value = true;

    if (this->existsQueue(id)) {
        this->lockQueue();
        const std::queue<Message *> *queue = messageQueue.find(id)->second;
        value = queue->empty();
        this->unlockQueue();
    }

    return value;
}

bool MemoryMessageCentral::existsQueue(const std::string id) const
{

    bool value = true;

    this->lockQueue();
    auto itr = messageQueue.find(id);

    if (itr == messageQueue.end()) {
        value = false;
    }
    this->unlockQueue();

    return value;
}

void MemoryMessageCentral::push(const std::string id, Message *message)
{

    if (!this->existsQueue(id)) {
        return;
    }

    this->lockQueue();
    std::queue<Message *> *queue = messageQueue[id];
    queue->push(message);
    this->unlockQueue();

}

Message* MemoryMessageCentral::pop(const std::string id)
{

    Message *value = NULL;

    if ((!this->existsQueue(id))) {
        return value;
    }

    this->lockQueue();
    std::queue<Message *> *queue = messageQueue.find(id)->second;
    if (!queue->empty()) {
        value = queue->front();
        queue->pop();
    }
    this->unlockQueue();

    return value;

}
