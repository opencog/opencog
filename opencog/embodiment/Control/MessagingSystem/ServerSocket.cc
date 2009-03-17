/**
 * ServerSocket.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 20:58:48 BRT 2007
 */

#include "ServerSocket.h"
#include <string.h> 

using namespace MessagingSystem;

NetworkElement *ServerSocket::master = NULL;

ServerSocket::~ServerSocket() {
    logger().log(opencog::Logger::DEBUG, "ServerSocket - Connection closed.");
}

ServerSocket::ServerSocket(ISocketHandler &handler):TcpSocket(handler) {

    logger().log(opencog::Logger::DEBUG, "ServerSocket - Serving connection.");
    // Enables "line-based" protocol, which will cause onLine() be called
    // everytime the peer send a line
    SetLineProtocol();
    SetSoKeepalive(); // trying to never close connection

    currentState = DOING_NOTHING;
    currentMessage.assign("");
}

void ServerSocket::setMaster(NetworkElement *ne) {
    master = ne;
}

void ServerSocket::OnLine(const std::string& line) {
    
//    logger().log(opencog::Logger::FINE, "ServerSocket - Received line: <%s>", line.c_str());
    char selector = line[0]; 
    std::string contents = line.substr(1);
    std::string command;

    if (selector == 'c') {
        logger().log(opencog::Logger::DEBUG, "ServerSocket - Received command: '%s'", contents.c_str());

        unsigned int pos1 = contents.find(' ', 0);
        if (pos1 == contents.npos) {
            command.assign(contents);
        } else {
            //command.assign(contents.substr(0, pos1));
            command = contents.substr(0, pos1);
        }

//        logger().log(opencog::Logger::DEBUG, "ServerSocket - Parsed command: <%s>",
 //                       command.c_str());

        if (command == "NOTIFY_NEW_MESSAGE") {
            //logger().log(opencog::Logger::DEBUG, "ServerSocket - Full line: <%s>", contents.c_str());
            std::string tmpStr = contents.substr(pos1 + 1);
            int numMessages = atoi(tmpStr.c_str());
            logger().log(opencog::Logger::DEBUG, "ServerSocket - Received notification of %u new message(s) from router", numMessages);
            master->newMessageInRouter(numMessages);
	    if (!master->noAckMessages) {
                //logger().log(opencog::Logger::DEBUG, "ServerSocket - Answering OK");
                Send("OK\n");
	    }

        } else if (command == "UNAVAILABLE_ELEMENT") {
            logger().log(opencog::Logger::DEBUG, 
                            "ServerSocket - Received UNAVAILABLE_ELEMENT message.");
            
            unsigned int pos2 = contents.find(' ', pos1 + 1);
            if (pos2 != contents.npos) {
                logger().log(opencog::Logger::WARNING, 
                    "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", 
                    line.c_str());
            }

            std::string id = contents.substr(pos1 + 1, pos2 - pos1 - 1);
            master->unavailableElement(id);

	    if (!master->noAckMessages) {
                logger().log(opencog::Logger::DEBUG, "ServerSocket - Answering OK");
                Send("OK\n");
	    }

        } else if (command == "AVAILABLE_ELEMENT") {
            logger().log(opencog::Logger::DEBUG, 
                            "ServerSocket - Received AVAILABLE_ELEMENT message.");
            
            unsigned int pos2 = contents.find(' ', pos1 + 1);
            if (pos2 != contents.npos) {
                logger().log(opencog::Logger::WARNING, 
                    "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", 
                    line.c_str());
            }

            std::string id = contents.substr(pos1 + 1, pos2 - pos1 - 1);
            master->availableElement(id);

	    if (!master->noAckMessages) {
                logger().log(opencog::Logger::DEBUG, "ServerSocket - Answering OK");
                Send("OK\n");
	    }

        } else if (command == "START_MESSAGE") {
            if (currentState == READING_MESSAGES) {
                master->newMessageRead(currentMessageFrom, currentMessageTo, currentMessageType, currentMessage);
                currentMessage.assign("");
                lineCount = 0;
                currentMessageFrom.assign("");
                currentMessageTo.assign("");
                currentMessageType = -1;
            } else {
                if (currentState == DOING_NOTHING) {
                    currentState = READING_MESSAGES;
                } else {
                    logger().log(opencog::Logger::WARNING, "ServerSocket - Unexpected command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
                }
            }
            unsigned int pos2 = contents.find(' ', pos1 + 1);
            if (pos2 == contents.npos) {
                logger().log(opencog::Logger::WARNING, "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", line.c_str());
            }
            unsigned int pos3 = contents.find(' ', pos2 + 1);
            if (pos3 == contents.npos) {
                logger().log(opencog::Logger::WARNING, "ServerSocket - Unknown command args. Discarding the entire line:\n\t%s", line.c_str());
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
            if (currentState == READING_MESSAGES) {
                master->newMessageRead(currentMessageFrom, currentMessageTo, currentMessageType, currentMessage);
                master->noMoreMessages();
                currentMessage.assign("");
                lineCount = 0;
                currentMessageFrom.assign("");
                currentMessageTo.assign("");
                currentMessageType = -1;
                currentState = DOING_NOTHING;
            } else {
                logger().log(opencog::Logger::WARNING, "ServerSocket - Unexpected command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
            }
            // Currently, router does not manage error during a message delivery. That's why we are sending
            // an OK even if above error occurs.
	    if (!master->noAckMessages) {
                //logger().log(opencog::Logger::DEBUG, "ServerSocket - Answering OK");
                Send("OK\n"); 
	    }

        } else {
            logger().log(opencog::Logger::WARNING, "ServerSocket - Unknown command (%s). Discarding the entire line:\n\t%s", command.c_str(), line.c_str());
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
            logger().log(opencog::Logger::WARNING, "ServerSocket - Unexpected data line. Discarding the entire line:\n\t%s", contents.c_str(), line.c_str());
        }
    } else {
        logger().log(opencog::Logger::WARNING, "ServerSocket - Invalid selection char ('%c') in received line. Expected 'c' or 'd'. Discarding the entire line:\n\t%s", selector, line.c_str());
    }
}

