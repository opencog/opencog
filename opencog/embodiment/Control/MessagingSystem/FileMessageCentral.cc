/*
 * opencog/embodiment/Control/MessagingSystem/FileMessageCentral.cc
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

#include "FileMessageCentral.h"

#include "stdio.h"
#include "stdlib.h"
#include <opencog/util/Logger.h>
#include <opencog/util/Config.h>
#include <opencog/util/files.h>

using namespace opencog::messaging;
using namespace opencog;


FileMessageCentral::~FileMessageCentral()
{
}

FileMessageCentral::FileMessageCentral() : MessageCentral()
{
    std::string dir = opencog::config().get("MESSAGE_DIR");
    expandPath(dir);

    logger().warn("FileMessageCentral - creating message dir: %s\n", dir.c_str());
    this->directory = dir;

    lockQueue();

    //check the directory
    if (!(exists(this->directory) || is_directory(this->directory))) {
        throw RuntimeException(TRACE_INFO,
                               "Parameter MESSAGE_DIR does not have real directory path: '%s'.",
                               this->directory.string().c_str());
    }
    this->directory /= ("fmq");

    //create a directory for messages
    if (!exists(this->directory)) {
        create_directory(this->directory);
    }
    unlockQueue();

}

void FileMessageCentral::createQueue(const std::string id, const bool reset)
{

    boost::filesystem::path m_path = this->directory / id;

    //test if id already exists in directory
    if (!this->existsQueue(id)) {
        this->lockQueue();
        create_directory(m_path);
        this->unlockQueue();
    } else {
        if (reset) {

            //I need delete all objects in the queue that already exists
            this->lockQueue();
            directory_iterator end_itr;
            for ( directory_iterator itr( m_path ); itr != end_itr; ++itr ) {
                remove(itr->path());
            }
            this->unlockQueue();
        }
    }
}

void FileMessageCentral::clearQueue(const std::string id)
{

    boost::filesystem::path m_path = this->directory / id;

    if (this->existsQueue(id)) {

        this->lockQueue();
        directory_iterator end_itr;
        for ( directory_iterator itr( m_path ); itr != end_itr; ++itr ) {
            remove(itr->path());
        }
    }
}


void FileMessageCentral::removeQueue(const std::string id)
{

    boost::filesystem::path m_path = this->directory / id;

    if (this->existsQueue(id)) {

        clearQueue(id);

        // remove directory
        remove(m_path);
        this->unlockQueue();
    }
}

bool FileMessageCentral::isQueueEmpty(const std::string id) const
{

    bool value = true;
    boost::filesystem::path m_path = this->directory / id;


    if (!this->existsQueue(id)) {
        return value;
    }

    directory_iterator end_itr;
    this->lockQueue();
    directory_iterator itr( m_path );
    //at least one file in directory
    if (itr != end_itr) {
        value = false;
    }
    this->unlockQueue();

    return value;
}

unsigned int FileMessageCentral::queueSize(const std::string id) const
{

    unsigned int value = 0;
    boost::filesystem::path m_path = this->directory / id;


    if (!this->existsQueue(id)) {
        return value;
    }

    directory_iterator end_itr;
    this->lockQueue();
    directory_iterator itr( m_path );
    //at least one file in directory

    while (itr != end_itr) {
        value ++;
        ++itr;
    }
    this->unlockQueue();

    return value;
}

bool FileMessageCentral::existsQueue(const std::string id) const
{

    bool value = true;

    boost::filesystem::path m_path = this->directory / id;

    this->lockQueue();
    if (!exists(m_path)) {
        value = false;
    }
    this->unlockQueue();

    return value;
}

void FileMessageCentral::push(const std::string id, Message *message)
{
    if (!this->existsQueue(id)) {
        delete message;
        return;
    }

    boost::filesystem::path m_path = this->directory / id;

    this->lockQueue();
    boost::posix_time::ptime this_time (boost::posix_time::microsec_clock::local_time());
    boost::filesystem::path file_name(boost::posix_time::to_iso_string(this_time));

    std::ofstream file( (m_path / file_name).string().c_str(), std::ios::out );
    file << message->getFrom() << "\n";
    file << message->getTo() << "\n";
    file << message->getPlainTextRepresentation();

    file.close();

    this->unlockQueue();
    delete message;

}

Message* FileMessageCentral::pop(const std::string id)
{

    Message *value = NULL;

    if ((!this->existsQueue(id)) || this->isQueueEmpty(id)) {
        return value;
    } else {
        boost::filesystem::path m_path = this->directory / id;
        std::string from("");
        std::string to("");
        std::string msg("");

        this->lockQueue();
        directory_iterator end_itr;
        directory_iterator itr( m_path );
        if ( itr != end_itr ) {
            std::string line;
            std::ifstream file( (itr->path()).string().c_str(), std::ios::in );

            std::getline(file, from);
            std::getline(file, to);

            while ( !file.eof()) {
                std::getline(file, line);
                msg.append(line.append("\n"));
// std::cout << "read line :" << line.c_str() << " size: " << line.size() << "\n";
            }

            // Erase the last \n put when reached at EOF.
            msg.erase(msg.length() - 1, 1);

// std::cout << "read :" << msg.c_str();
            file.close();

            value = new StringMessage(from, to, msg);

            remove(itr->path());

        }
        this->unlockQueue();
    }

    return value;

}


