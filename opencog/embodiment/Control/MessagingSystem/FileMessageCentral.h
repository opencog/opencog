/*
 * opencog/embodiment/Control/MessagingSystem/FileMessageCentral.h
 *
 * Copyright (C) 2007-2008 Elvys Borges
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

#include <SystemParameters.h>
#include <opencog/util/exceptions.h>
#include "StringMessage.h"

#include <fstream>
#include <iostream>
#include <exception>

namespace MessagingSystem
{

using namespace boost::filesystem;


/**
 * Implements MessageCentral using a map of queue, in memory.
 *
 */
class FileMessageCentral : public MessageCentral
{

private:

    const Control::SystemParameters& parameters;
    path directory;

public:

    /**
     * Constructors and destructor
     */
    ~FileMessageCentral();

    FileMessageCentral(const Control::SystemParameters &params) throw (RuntimeException, std::bad_exception);

    void createQueue(const std::string id, const bool reset = false);

    void clearQueue(const std::string id);

    void removeQueue(const std::string id);

    const bool isQueueEmpty(const std::string id);

    const int sizeQueue(const std::string id);

    const bool existsQueue(const std::string id);

    void push(const std::string id, Message *message);

    Message* pop(const std::string id);

}; // class
}  // namespace

#endif
