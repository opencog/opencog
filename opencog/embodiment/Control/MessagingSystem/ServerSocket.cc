/*
 * opencog/embodiment/Control/MessagingSystem/ServerSocket.cc
 *
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

#include "ServerSocket.h"
#include <string.h>

#include <boost/bind.hpp>

using namespace opencog::messaging;
using namespace opencog;

NetworkElement *ServerSocket::master = NULL;

ServerSocket::~ServerSocket()
{
    socket.close();
    logger().debug("ServerSocket destroyed - Connection closed.");
}

ServerSocket::ServerSocket() : socket(io_service)
{
    logger().debug("ServerSocket - Serving connection.");

    currentState = DOING_NOTHING;
    currentMessage.assign("");
}

void ServerSocket::setMaster(NetworkElement *ne)
{
    master = ne;
}

tcp::socket& ServerSocket::getSocket() 
{
    return socket;
}

void ServerSocket::start()
{
    connectionThread = boost::thread(boost::bind(&handle_connection, this)); 
}

void ServerSocket::handle_connection(ServerSocket* ss)
{
    logger().debug("ServerSocket::handle_connection()");
    boost::asio::streambuf b;
    for (;;) 
    {
        try {
        logger().debug("%p: ServerSocket::handle_connection(): Called read_until", ss);
        boost::asio::read_until(ss->getSocket(), b, '\n');
        logger().debug("%p: ServerSocket::handle_connection(): returned from read_until", ss);
        std::istream is(&b);
        std::string line;
        std::getline(is, line); 
        if (!line.empty() && line[line.length()-1] == '\r') {
            line.erase(line.end()-1);
        }
        logger().debug("%p: ServerSocket::handle_connection(): Got new line: %s", ss, line.c_str());

        // Don't attempt to process empty lines, because the processing logic blindly uses substrings:
        if (!line.empty())
            ss->OnLine(line);

        } catch (boost::system::system_error& e) {
            if (ss->master->isListenerThreadStopped()) {
                break;
            } else {
                logger().error("ServerSocket::handle_connection(): Error reading data. Message: %s", e.what());
            }
        }
    }
}

void ServerSocket::Send(const std::string& cmd)
{
    boost::system::error_code error;
    boost::asio::write(socket, boost::asio::buffer(cmd), boost::asio::transfer_all(), error);
    if (error) {
        logger().error("ServerSocket::Send(): Error transfering data.");
    }
}

void ServerSocket::OnLine(const std::string& line)
{

    logger().debug("ServerSocket - Received line: <%s>", line.c_str());
    char selector = line[0];
    std::string contents = line.substr(1);
    std::string command;

    if (selector == 'c') {
        logger().debug("ServerSocket - Received command: '%s'", contents.c_str());

        size_t pos1 = contents.find(' ', 0);
        if (pos1 == contents.npos) {
            command.assign(contents);
        } else {
            //command.assign(contents.substr(0, pos1));
            command = contents.substr(0, pos1);
        }

        logger().debug("ServerSocket - Parsed command: <%s>",
                       command.c_str());

        if (command == "NOTIFY_NEW_MESSAGE") {
            logger().debug("ServerSocket - Full line: <%s>", contents.c_str());
            std::string tmpStr = contents.substr(pos1 + 1);
            int numMessages = atoi(tmpStr.c_str());
            logger().debug("ServerSocket - Received notification of %u new message(s) from router", numMessages);
            master->newMessageInRouter(numMessages);
            if (!master->noAckMessages) {
                logger().debug("ServerSocket - Answering OK");
                Send("OK\n");
            }

        } else if (command == "UNAVAILABLE_ELEMENT") {
            logger().debug(
                         "ServerSocket - Received UNAVAILABLE_ELEMENT message.");

            size_t pos2 = contents.find(' ', pos1 + 1);
            if (pos2 != contents.npos) {
                logger().warn(
                             "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s",
                             line.c_str());
                return;
            }

            std::string id = contents.substr(pos1 + 1, pos2 - pos1 - 1);
            master->unavailableElement(id);

            if (!master->noAckMessages) {
                logger().debug("ServerSocket - Answering OK");
                Send("OK\n");
            }

        } else if (command == "AVAILABLE_ELEMENT") {
            logger().debug(
                         "ServerSocket - Received AVAILABLE_ELEMENT message.");

            size_t pos2 = contents.find(' ', pos1 + 1);
            if (pos2 != contents.npos) {
                logger().warn(
                             "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s",
                             line.c_str());
                return;
            }

            std::string id = contents.substr(pos1 + 1, pos2 - pos1 - 1);
            master->availableElement(id);

            if (!master->noAckMessages) {
                logger().debug("ServerSocket - Answering OK");
                Send("OK\n");
            }

        } else if (command == "START_MESSAGE") {
            logger().debug("ServerSocket - Handling START_MESSAGE command");
            if (currentState == READING_MESSAGES) {
                logger().debug("ServerSocket - Calling newMessageRead");
                master->newMessageRead(currentMessageFrom, currentMessageTo, currentMessageType, currentMessage);
                logger().debug("ServerSocket - Cleaning control variables");
                currentMessage.assign("");
                lineCount = 0;
                currentMessageFrom.assign("");
                currentMessageTo.assign("");
                currentMessageType = -1;
            } else {
                if (currentState == DOING_NOTHING) {
                    logger().debug("ServerSocket - Starting new message reading");
                    currentState = READING_MESSAGES;
                } else {
                    logger().warn("ServerSocket - Unexpected command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
                    return;
                }
            }
            size_t pos2 = contents.find(' ', pos1 + 1);
            if (pos2 == contents.npos) {
                logger().warn("ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", line.c_str());
                return;
            }
            size_t pos3 = contents.find(' ', pos2 + 1);
            if (pos3 == contents.npos) {
                logger().warn("ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", line.c_str());
                return;
            }
            //currentMessageFrom.assign(contents.substr(pos1 + 1, pos2 - pos1 - 1));
            currentMessageFrom = contents.substr(pos1 + 1, pos2 - pos1 - 1);
            //currentMessageTo.assign(contents.substr(pos2 + 1, pos3 - pos2 - 1));
            currentMessageTo = contents.substr(pos2 + 1, pos3 - pos2 - 1);
            //currentMessageType = atoi(contents.substr(pos3 + 1).c_str());
            std::string tmpStr = contents.substr(pos3 + 1);
            currentMessageType = atoi(tmpStr.c_str());
            lineCount = 0;

        } else if (command == "NO_MORE_MESSAGES") {
            logger().debug("ServerSocket - Handling NO_MORE_MESSAGES command");
            if (currentState == READING_MESSAGES) {
                logger().debug("ServerSocket - Starting new message reading");
                master->newMessageRead(currentMessageFrom, currentMessageTo, currentMessageType, currentMessage);
                master->noMoreMessages();
                currentMessage.assign("");
                lineCount = 0;
                currentMessageFrom.assign("");
                currentMessageTo.assign("");
                currentMessageType = -1;
                currentState = DOING_NOTHING;
            } else {
                logger().warn("ServerSocket - Unexpected command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
                return;
            }
            // Currently, router does not manage error during a message delivery. That's why we are sending
            // an OK even if above error occurs.
            if (!master->noAckMessages) {
                logger().debug("ServerSocket - Answering OK");
                Send("OK\n");
            }

        } else {
            logger().warn("ServerSocket - Unknown command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
            return;
        }

    } else if (selector == 'd') {
        // The line is part of a message
        if (currentState == READING_MESSAGES) {
            if (lineCount > 0) {
                currentMessage.append("\n");
            }
            currentMessage.append(contents);
            lineCount++;
        } else {
            logger().warn("ServerSocket - Unexpected data line. Discarding the entire line:\n\t%s", contents.c_str(), line.c_str());
            return;
        }
    } else {
        logger().warn("ServerSocket - Invalid selection char ('%c') in received line. Expected 'c' or 'd'. Discarding the entire line:\n\t%s", selector, line.c_str());
        return;
    }
}

