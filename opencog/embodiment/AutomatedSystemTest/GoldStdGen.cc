/*
 * opencog/embodiment/AutomatedSystemTest/GoldStdGen.cc
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


#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencog/util/Logger.h>

#include <opencog/embodiment/Control/MessagingSystem/MessageFactory.h>
#include "GoldStdGen.h"

using namespace opencog::messaging;
using namespace AutomatedSystemTest;

GoldStdGen::GoldStdGen(const char* goldStdFilename)
{
    file = fopen(goldStdFilename, "w");
    if (file) {
        logger().info("Generating gold standard in file: %s", goldStdFilename);
    } else {
        logger().error("Could not open gold standard file: %s", goldStdFilename);
    }
    initial_time = getCurrentTimestamp();
}

GoldStdGen::~GoldStdGen()
{
    if (file) {
        fclose(file);
        fflush(file);
    }
}


void GoldStdGen::writeMessage(opencog::messaging::Message& message, bool sending)
{
    if (file) {
        fprintf(file, "%s%lu\n%s %s %d\n%s\n%s",
                sending ? SENT_MESSAGE_FLAG : RECEIVED_MESSAGE_FLAG,
                getCurrentTimestamp() - initial_time,
                message.getFrom().c_str(),
                message.getTo().c_str(),
                message.getType(),
                message.getPlainTextRepresentation(),
                MESSAGE_END_FLAG);
        fflush(file);
    }
}

GoldStdMessage* GoldStdGen::readMessage(char* line_buf, size_t lineBufSize, FILE* file)
{
    // read timestamp
    if (fgets(line_buf, lineBufSize, file) == NULL) {
        logger().error("Unexpected end of file while reading message's header");
        return NULL;
    }
    unsigned long timestamp;
    sscanf(line_buf, "%lu", &timestamp);

    // read message header
    if (fgets(line_buf, lineBufSize, file) == NULL) {
        logger().error("Unexpected end of file while reading message's header");
        return NULL;
    }
    char from[64];
    char to[64];
    int type;
    sscanf(line_buf, "%s %s %d", from, to, &type);

    // read the message body
    std::string message;
    while (true) {
        if (fgets(line_buf, lineBufSize, file) == NULL) {
            logger().error("Unexpected end of file while reading message: %s", message.c_str());
            return NULL;
        }
        if (!strcmp(MESSAGE_END_FLAG, line_buf)) {
            break;
        }
        message += line_buf;
    }
    // check and remove the '\n' added by writeMessage() method at the end of the message
    if (message.at(message.length() - 1) != '\n') {
        logger().error("Failed reading message, which should terminate with <new line> character: %s", message.c_str());
        exit(-1);
    }
    message.erase(message.length() - 1);

    return new GoldStdMessage(timestamp, messageFactory(from, to, type, message));
}

unsigned long GoldStdGen::getCurrentTimestamp()
{
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    std::string timeStr = to_iso_extended_string(now);
    //cout << "Current date/time = " << timeStr << endl;
    return opencog::pai::PAI::getTimestampFromXsdDateTimeStr(timeStr.c_str());
}

