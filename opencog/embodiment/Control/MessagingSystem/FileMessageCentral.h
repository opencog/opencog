/*
 * opencog/embodiment/Control/MessagingSystem/FileMessageCentral.h
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

#ifndef FILEMESSAGECENTRAL_H_
#define FILEMESSAGECENTRAL_H_

#include "MessageCentral.h"

#include "boost/filesystem.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <opencog/util/exceptions.h>
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>

#include <fstream>
#include <iostream>
#include <exception>

namespace opencog { namespace messaging {

using namespace boost::filesystem;


/**
 * Implements MessageCentral using a map of queue, in memory.
 *
 */
class FileMessageCentral : public MessageCentral
{

private:

    path directory;

public:

    /**
     * Constructors and destructor
     */
    ~FileMessageCentral();
    FileMessageCentral();

    void createQueue(const std::string id, const bool reset = false);

    void clearQueue(const std::string id);

    void removeQueue(const std::string id);

    bool isQueueEmpty(const std::string id) const;

    unsigned int queueSize(const std::string id) const;

    bool existsQueue(const std::string id) const;

    void push(const std::string id, Message *message);

    Message* pop(const std::string id);

}; // class
} } // namespace opencog::messaging

#endif
