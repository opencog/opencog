/*
 * opencog/embodiment/Control/MessagingSystem/RouterServerSocket.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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

#include <sstream>
#include <string.h>

#include <boost/bind.hpp>

#include <opencog/util/Logger.h>
#include <opencog/util/StringManipulator.h>

#include "Message.h"
#include "MessageFactory.h"
#include "RouterMessage.h"
#include "RouterServerSocket.h"
#include "NetworkElementCommon.h"


using namespace opencog::messaging;
using namespace opencog;

Router *RouterServerSocket::master = NULL;
// Disable sending notification messages and just route messages immediately.
bool no_msg_arrival_notification = true;

RouterServerSocket::~RouterServerSocket()
{
    socket.close();
    logger().debug("RouterServerSocket[%p] - destroyed. Connection closed.", this);
}

RouterServerSocket::RouterServerSocket() : socket(io_service)
{
    logger().debug("RouterServerSocket[%p] - Serving connection", this);
    currentState = WAITING_COMMAND;
    logger().debug("RouterServerSocket - CurrentState = %d", currentState);
}

void RouterServerSocket::setMaster(Router *router)
{
    master = router;
}

tcp::socket& RouterServerSocket::getSocket() 
{
    return socket;
}

void RouterServerSocket::start()
{
    connectionThread = boost::thread(boost::bind(&handle_connection, this)); 
}

void RouterServerSocket::handle_connection(RouterServerSocket* ss)
{
    logger().debug("RouterServerSocket::handle_connection()");
    boost::asio::streambuf b;
    for (;;) 
    {
        try {
        //logger().debug("%p: RouterServerSocket::handle_connection(): Called read_until", ss);
        boost::asio::read_until(ss->getSocket(), b, boost::regex("\n"));
        //logger().debug("%p: RouterServerSocket::handle_connection(): returned from read_until", ss);
        std::istream is(&b);
        std::string line;
        std::getline(is, line); 
        if (!line.empty() && line[line.length()-1] == '\r') {
            line.erase(line.end()-1);
        }
        //logger().debug("%p: RouterServerSocket::handle_connection(): Got new line: %s", ss, line.c_str());
        ss->OnLine(line);
        } catch (boost::system::system_error& e) {
            if (ss->master->isListenerThreadStopped()) {
                break;
            } else {
                logger().error("RouterServerSocket::handle_connection(): Error reading data. Message: %s", e.what());
            }
        }
    }
}

void RouterServerSocket::Send(const std::string& cmd)
{
    boost::system::error_code error;
    boost::asio::write(socket, boost::asio::buffer(cmd),
                       boost::asio::transfer_all(), error);
    if (error) {
        logger().error("RouterServerSocket::Send(): Error transfering data.");
    }
}

void RouterServerSocket::sendAnswer(const std::string &msg)
{
    logger().debug("RouterServerSocket - Answering: %s", msg.c_str());
    if (!master->noAckMessages) {
        Send(msg + "\n");
    }
}

void RouterServerSocket::addNetworkElement(const std::string &id,
                                           const std::string &ip,
                                           int port)
{
    logger().debug("RouterServerSocket - Add network element (%s,%s,%d)",
                   id.c_str(), ip.c_str(), port);

    char errorCode = master->addNetworkElement(id, ip, port);
    std::string answer;

    if (errorCode == Router::PORT_EXISTS) {
        answer = "FAILED - port number already exists: " + opencog::toString(port);
    } else if (errorCode == Router::ID_EXISTS) {
        answer = "FAILED - id already exists: " +  answer.append(id);
    } else if (errorCode == Router::NO_ERROR ||
               errorCode == Router::HAS_PENDING_MSGS) {
        answer = NetworkElementCommon::OK_MESSAGE;
    } else {
        answer = "FAILED - unknown error code.";
    }
    sendAnswer(answer);

    // sending pending messages stored in component queue
    if (errorCode == Router::HAS_PENDING_MSGS) {

        // notify about messages in queue
        //
        // IMPORTANT:
        // Since the component has just done the handshaking process the
        // chance to fail to receive the notification is null. If this
        // proves to be wrong then get the sender of the first message in
        // the queue.
        unsigned int numMessages = master->getMessageCentral()->queueSize(id);
        logger().debug("RouterServerSocket - Number of pending messages to %s: %u", id.c_str(), numMessages);
        if (no_msg_arrival_notification) {
            sendRequestedMessages(id, numMessages);
        } else {
            master->notifyMessageArrival(id, numMessages);
        }
    }
}

void RouterServerSocket::sendRequestedMessages(const std::string &id, int limit)
{

    std::string answer;
    char s[256];

    logger().debug("RouterServerSocket - Network element %s requested %d messages", id.c_str(), limit);

    bool known = master->knownID(id);

    if (known) {
        logger().fine("RouterServerSocket - known destination");
        answer = NetworkElementCommon::OK_MESSAGE;
        sendAnswer(answer);
    } else {
        logger().warn("RouterServerSocket - unknow destination");
        answer = "FAILED - could not send messages. Unknown ID: " +  id;
        sendAnswer(answer);
        return;
    }

    if (master->getMessageCentral()->isQueueEmpty(id)) {
        logger().warn("RouterServerSocket - There is no message to %s", id.c_str());
        return;
    }

    logger().info("RouterServerSocket - Delivering messages to %s", id.c_str());
    if (!master->dataSocketConnection(id)) {
        return;
    }
    tcp::socket* sock = master->getDataSocket(id);

    std::string cmd;
    while (limit != 0 && !master->getMessageCentral()->isQueueEmpty(id)) {

        // Remember that all message stored within the router are of
        // RouterMessage type.
        RouterMessage *message = (RouterMessage *)master->getMessageCentral()->pop(id);
        sprintf(s, "cSTART_MESSAGE %s %s %d", message->getFrom().c_str(),
                message->getTo().c_str(), message->getEncapsulateType());
        logger().debug("RouterServerSocket - Sending message (socket = %p): <%s>", sock, s);

        cmd = std::string(s) + "\n";
        unsigned int sentBytes = boost::asio::write(*sock, boost::asio::buffer(cmd));
        if (sentBytes != cmd.length()) {
            logger().error("RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, cmd.length() );
            master->closeSockets(id);
            master->markElementUnavailable(id);
            delete(message); // TODO: Shouldn't message be put back to the queue?
            return;
        }

        std::istringstream stream(message->getPlainTextRepresentation());
        std::string line;
        while (getline(stream, line)) {
            logger().debug("Sending line <d%s>", line.c_str());
            line.insert(0, "d");
            line.append("\n");
            sentBytes = boost::asio::write(*sock, boost::asio::buffer(line));
            if (sentBytes != line.length()) {
                logger().error("RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, line.length() );
                master->closeSockets(id);
                master->markElementUnavailable(id);
                delete(message); // TODO: Shouldn't message be put back to the queue?
                return;
            }
            // TODO: Shouldn't the protocol be changed to expect a
            // feedback (OK or FAILED) per message here?
            // And, if OK is not received, message may be kept on the
            // queue (until it be eventually sent or, at least, until
            // N attempts are made)
        }
        delete(message);
        if (limit > 0) limit--;
    }
    cmd = "cNO_MORE_MESSAGES\n";
    unsigned int sentBytes = boost::asio::write(*sock, boost::asio::buffer(cmd));
    if (sentBytes != cmd.length()) {
        logger().error("RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, cmd.length() );
        master->closeSockets(id);
        master->markElementUnavailable(id);
        return;
    }

    if (!master->noAckMessages) {
        logger().debug("RouterServerSocket - Waiting OK (after sending message).");
#define BUFFER_SIZE 256
        char response[BUFFER_SIZE];
        boost::system::error_code error;
        size_t receivedBytes = sock->read_some(boost::asio::buffer(response), error);
        if (error && error != boost::asio::error::eof) {
            logger().error("RouterServerSocket - Invalid response. recv returned %d ", receivedBytes );
            master->closeSockets(id);
            master->markElementUnavailable(id);
            return;
        }

        response[receivedBytes] = '\0'; // Assure null terminated string
        // chomp all trailing slashes from string
        int i;
        for ( i = receivedBytes - 1; i >= 0 && (response[i] == '\n' || response[i] == '\r')  ; --i ) {
            response[i] = '\0';
        }

        logger().debug("RouterServerSocket - Received response (after chomp): '%s' bytes: %d",
                       response, receivedBytes );
        std::string answer = response;

        if (answer == NetworkElementCommon::OK_MESSAGE) {
            logger().debug("RouterServerSocket - Sucessfully sent messages to '%s'.",
                           id.c_str());
        } else {
            logger().error("RouterServerSocket - Failed to send messages to '%s'. (answer = %s)",
                           id.c_str(), answer.c_str());
        }
    }

    logger().debug("RouterServerSocket - id <%s> message queue size %d",
                   id.c_str(), master->getMessageCentral()->queueSize(id));

    logger().debug("RouterServerSocket - OK");
}

void RouterServerSocket::storeNewMessage()
{

    std::string answer;
    bool addToMasterQueue = true;
    bool notifyMessageArrival = true;

    logger().debug("RouterServerSocket - New message arrived. "
                   "From '%s'. To '%s'. Type '%d'. Lines: %d",
                   currentMessageFrom.c_str(), currentMessageTo.c_str(),
                   currentMessageType, currentMessageSize);
    logger().debug("RouterServerSocket - Message body:\n%s",
                   currentMessageText.c_str());

    bool knownFrom = master->knownID(currentMessageFrom);
    bool knownTo   = master->knownID(currentMessageTo);

    if (!knownFrom) {
        logger().debug("RouterServerSocket - Unknown from '%s'. "
                       "Discarding message.", currentMessageFrom.c_str());
        answer = "FAILED - Unknown <from> ID: " + currentMessageFrom;
        sendAnswer(answer);
        return;

    } else if (!knownTo) {
        logger().debug("RouterServerSocket - Unknown to '%s' (from '%s'). "
                       "Wont be notified.", currentMessageTo.c_str(),
                       currentMessageFrom.c_str());

        // since the component is not registred there is no need to notify it about new messages
        notifyMessageArrival = false;

        // Marks this element as unavailable so that Router notifies all other elements
        // that eventually has this element as available.
        master->markElementUnavailable(currentMessageTo);

        //answer.assign("FAILED - Unknown <to> ID: "); answer.append(currentMessageTo);
        //sendAnswer(answer);
        //return;
    } else if (!master->isElementAvailable(currentMessageTo)) {
        logger().warn("RouterServerSocket - Element '%s' unavailable. "
                      "Discarding message.", currentMessageTo.c_str());
        answer = "FAILED - Element unavailable: " + currentMessageTo;
        sendAnswer(answer);
        return;
    }

    if (addToMasterQueue) {

//        try {

        // Router has its own message type (RouterMessage) to
        // encapsulate all messages received. It is not Router's duty
        // to check if a message is well formed or not. This should be
        // done by the NetworkElement receiving the message.



        logger().debug("RouterServerSocket - Building Message object.");
        Message *message = routerMessageFactory(currentMessageFrom, currentMessageTo,
                                                currentMessageType, currentMessageText);

        logger().debug("RouterServerSocket - Queueing message.");
        master->getMessageCentral()->push(currentMessageTo, message);

//        } catch (InvalidParamException& e){
//         logger().error(
//                "RouterServerSocket - Discarding message with invalid parameter.\nMessage:\n%s.",
//                currentMessageText.c_str());
//        }
    }

    // Ack sender that message was delivered successfully
    answer = NetworkElementCommon::OK_MESSAGE;
    logger().debug("RouterServerSocket - Sending OK.");
    sendAnswer(answer);

    if (notifyMessageArrival) {
        if (no_msg_arrival_notification) {
            sendRequestedMessages(currentMessageTo, -1);
        } else {
            master->notifyMessageArrival(currentMessageTo, 1);
        }
    }
    logger().fine("RouterServerSocket - Finished queueing messages.");
}

void RouterServerSocket::OnLine(const std::string& line)
{

    logger().debug("RouterServerSocket[%p] - Received line <%s>",
                   this, line.c_str());
    logger().debug("RouterServerSocket - State = %d", currentState);

    switch (currentState) {
    case WAITING_COMMAND: {
        std::string command;
        std::queue<std::string> args;
        NetworkElementCommon::parseCommandLine(line, command, args);
        logger().debug("RouterServerSocket - Parsed command <%s>. Args size: %d.", command.c_str(), args.size());
        if (command == "LOGIN") {
            // handshake
            std::string id = args.front();
            args.pop();
            std::string ip = args.front();
            args.pop();
            int port = atoi(args.front().c_str());
            args.pop();

            logger().info("RouterServerSocket - Handshaking: id = %s ip = %s port = %d.",
                          id.c_str(), ip.c_str(), port);

            addNetworkElement(id, ip, port);

        } else if (command == "LOGOUT") {
            // logout an NetworkElement - most of the time an OAC
            std::string id = args.front();
            args.pop();

            logger().info("RouterServerSocket - Logout: id = %s.", id.c_str());

            master->removeNetworkElement(id);
            sendAnswer(NetworkElementCommon::OK_MESSAGE);

        } else if (command == "CLEAR_MESSAGE_QUEUE") {
            // request the removal of all messages from a NE message queue
            // IMPORTANT: this command should be used only by SPAWNER
            std::string requestorId = args.front();
            args.pop();

            if (requestorId != opencog::config().get("SPAWNER_ID")) {
                sendAnswer("FAILED - Requestor not SPAWNER.");
                return;
            }

            std::string targetId = args.front();
            args.pop();

            logger().info("RouterServerSocket - Clear message queue: id = '%s' requesting for '%s'.",
                          requestorId.c_str(), targetId.c_str());

            master->clearNetworkElementMessageQueue(targetId);
            sendAnswer(NetworkElementCommon::OK_MESSAGE);

        } else if (command == "SHUTDOWN") {
            // exit
            logger().info("RouterServerSocket - Shutdown router.");
            master->shutdown();

        } else if (command == "REQUEST_UNREAD_MESSAGES") {
            // request messages
            std::string id = args.front();
            args.pop();
            int limit = atoi(args.front().c_str());
            args.pop();
            logger().info("RouterServerSocket - %s requested messages (limit = %d).", id.c_str(), limit);
            sendRequestedMessages(id, limit);

        } else if (command == "NEW_MESSAGE") {
            // new message arrived (to be stored in router and delivered under request)
            currentMessageFrom = args.front();
            args.pop();
            currentMessageTo = args.front();
            args.pop();
            currentMessageType = atoi(args.front().c_str());
            args.pop();
            currentMessageSize = atoi(args.front().c_str());
            args.pop();
            currentMessageText.assign("");
            currentState = READING_MESSAGE;
            firstLineOfMessageFlag = true;
            messageLinesCountdown = currentMessageSize;
            logger().info("RouterServerSocket - Message arriving. From '%s' To '%s'.", currentMessageFrom.c_str(), currentMessageTo.c_str());
        } else {
            logger().error("RouterServerSocket - Invalid command (%s) received. Discarding it.", command.c_str());
        }
        break;
    }
    case READING_MESSAGE: {
        if (firstLineOfMessageFlag) {
            firstLineOfMessageFlag = false;
        } else {
            currentMessageText.append("\n");
        }
        currentMessageText.append(line);
        messageLinesCountdown--;
        if (messageLinesCountdown == 0) {
            logger().debug("RouterServerSocket - Message complete.");
            storeNewMessage();
            currentState = WAITING_COMMAND;
        }
        break;
    }
    default: {
        logger().warn("RouterServerSocket - Invalid state (%d). Comming back to WAITING_COMMAND (discarded last line).", currentState);
        currentState = WAITING_COMMAND;
        break;
    }
    }
}

