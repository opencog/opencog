/*
 * opencog/embodiment/Control/MessagingSystem/MemoryMessageCentral.h
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

#ifndef MEMORYMESSAGECENTRAL_H_
#define MEMORYMESSAGECENTRAL_H_

#include "MessageCentral.h"

#include <map>
#include <queue>

namespace opencog { namespace messaging {

/**
 * Implements MessageCentral using a map of queue, in memory.
 *
 * @see MessageCentral parent class for class documentation.
 *
 */
class MemoryMessageCentral : public MessageCentral
{

private:

    std::map<std::string, std::queue<Message *> *> messageQueue;

public:

    /**
     * Constructors and destructor
     */
    ~MemoryMessageCentral();
    MemoryMessageCentral();

    void createQueue(const std::string id, const bool reset = false);

    void clearQueue(const std::string id);

    void removeQueue(const std::string id);

    unsigned int queueSize(const std::string id) const;

    bool isQueueEmpty(const std::string id) const;

    bool existsQueue(const std::string id) const;

    void push(const std::string id, Message *message);

    Message* pop(const std::string id);

}; // class
} } // namespace opencog::messaging

#endif
