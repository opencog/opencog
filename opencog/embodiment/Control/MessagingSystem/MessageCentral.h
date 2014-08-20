/*
 * opencog/embodiment/Control/MessagingSystem/MessageCentral.h
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

#ifndef MESSAGECENTRAL_H
#define MESSAGECENTRAL_H

#include <pthread.h>
#include <string>
#include "Message.h"

namespace opencog { namespace messaging {

/**
 * Interface implemented by classes which deal with Message Queue
 */
class MessageCentral
{

protected:

    void lockQueue() const;
    void unlockQueue() const;


private:

    //! lock used coordinate manipulation of the messageQueue (main thread and listener thread)
    mutable pthread_mutex_t messageQueueLock;

public:

    virtual ~MessageCentral();
    MessageCentral();

    /** Create a internal queue.
     * if reset == true and the queue already exists, it will be cleaned.
     * @param id representation of queue name
     * @param reset flag to clear queue if it already exists
     */
    virtual void createQueue(const std::string id, const bool reset = false) = 0;

    /**
     * Remove all messages from internal queue.
     * @param id representing the queue name
     */
    virtual void clearQueue(const std::string id) = 0;

    /**
     * Remove a internal queue.
     * @param id representation of queue name
     */
    virtual void removeQueue(const std::string id) = 0;

    /**
     * Check if the queue is empty
     * @param id representation of queue name
     */
    virtual bool isQueueEmpty(const std::string id) const = 0;

    /**
     * Gets the size of a given queue
     * @param id representation of queue name
     */
    virtual unsigned int queueSize(const std::string id) const = 0;

    /**
     * Check if the queue exists
     * @param id representation of queue name
     */
    virtual bool existsQueue(const std::string id) const = 0;

    /**
     * Put a message in a queue
     * @param id is the queue name
     * @param message is the message
     */
    virtual void push(const std::string id, Message *message) = 0;

    /**
     * Get the oldest messagem in queue
     * @param id is the queue name
     * @return a message
     */
    virtual Message* pop(const std::string id) = 0;

}; // class
} } // namespace opencog::messaging

#endif
