/*
 * opencog/embodiment/Control/MessagingSystem/RouterServerSocket.cc
 *
 * Copyright (C) 2007-2008 Andre Senna
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

#include "Message.h"
#include "RouterMessage.h"
#include "RouterServerSocket.h"
#include "NetworkElement.h"

#include "util/Logger.h"
#include "util/StringManipulator.h"

#include <Sockets/SocketHandler.h>
#include <Sockets/StdoutLog.h>

#include <sstream>
#include <string.h>
#include <RouterHttpPostSocket.h>


using namespace MessagingSystem;
using namespace opencog;

Router *RouterServerSocket::master = NULL;
bool no_msg_arrival_notification = false; // TEST: unfortunately it didn't work well when set to true. Needs more testing and debbuging.

RouterServerSocket::~RouterServerSocket()
{
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket[%p] - destroyed. Connection closed.", this);
}

RouterServerSocket::RouterServerSocket(ISocketHandler &handler): TcpSocket(handler)
{

    logger().log(opencog::Logger::DEBUG, "RouterServerSocket[%p] - Serving connection", this);
    currentState = WAITING_COMMAND;
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - CurrentState = %d", currentState);

    // Enables "line-based" protocol, which will cause onLine() be called
    // everytime the peer send a line
    SetLineProtocol();
    SetSoKeepalive(); // trying to never close connection
}

void RouterServerSocket::setMaster(Router *router)
{
    master = router;
}

void RouterServerSocket::sendAnswer(const std::string &msg)
{
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Answering: %s", msg.c_str());
    if (!master->noAckMessages) {
        Send(msg + "\n");
    }
}

void RouterServerSocket::addNetworkElement(const std::string &id, const std::string &ip, int port)
{
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Add network element (%s,%s,%d)", id.c_str(), ip.c_str(), port);

    char errorCode = master->addNetworkElement(id, ip, port);
    std::string answer;

    if (errorCode == Router::PORT_EXISTS) {
        answer.assign("FAILED - port number already exists: "); answer.append(opencog::toString(port));
    } else if (errorCode == Router::ID_EXISTS) {
        answer.assign("FAILED - id already exists: "); answer.append(id);
    } else if (errorCode == Router::NO_ERROR || errorCode == Router::HAS_PENDING_MSGS) {
        answer.assign(NetworkElement::OK_MESSAGE);
    } else {
        answer.assign("FAILED - unknown error code.");
    }
    sendAnswer(answer);

    // sending pending messages stored in component queue
    if (errorCode == Router::HAS_PENDING_MSGS) {

        if (id == opencog::config().get("PROXY_ID") &&
                master->getPortNumber(id) == 8211) {
            // TODO: this is a temporary hack so that we can use both proxy and mocky proxy

            // Send all pending messages to the router
            sendRequestedMessages(id, -1, true);
        } else {
            // notify about messages in queue
            //
            // IMPORTANT:
            // Since the component has just done the handshaking process the
            // chance to fail to receive the notification is null. If this
            // proves to be wrong then get the sender of the first message in
            // the queue.
            unsigned int numMessages = master->getMessageCentral()->queueSize(id);
            logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Number of pending messages to %s: %u", id.c_str(), numMessages);
            if (no_msg_arrival_notification) {
                sendRequestedMessages(id, numMessages);
            } else {
                master->notifyMessageArrival(id, numMessages);
            }
        }
    }
}

void RouterServerSocket::sendRequestedMessages(const std::string &id, int limit, bool useHttpRequest)
{

    std::string answer;
    char s[256];

    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Network element %s requested %d messages", id.c_str(), limit);

    bool known = master->knownID(id);

    if (known) {
        logger().log(opencog::Logger::FINE, "RouterServerSocket - known destination");
        answer.assign(NetworkElement::OK_MESSAGE);
        sendAnswer(answer);
    } else {
        logger().log(opencog::Logger::WARN, "RouterServerSocket - unknow destination");
        answer.assign("FAILED - could not send messages. Unknown ID: "); answer.append(id);
        sendAnswer(answer);
        return;
    }

    if (master->getMessageCentral()->isQueueEmpty(id)) {
        logger().log(opencog::Logger::WARN, "RouterServerSocket - There is no message to %s", id.c_str());
        return;
    }

    logger().log(opencog::Logger::INFO, "RouterServerSocket - Delivering messages to %s", id.c_str());
    if (useHttpRequest) {
        while (limit != 0 && !master->getMessageCentral()->isQueueEmpty(id)) {

            Message *message = master->getMessageCentral()->pop(id);
            if (message->getType() != Message::STRING) {
                logger().log(opencog::Logger::WARN,
                             "RouterServerSocket - HttpRequest support STRING messages. Discarding message type: %d.",
                             message->getType());

                // TODO: convert non Message::STRING types on the fly
                continue;
            }

            sendHttpRequest(message->getFrom().c_str(), message->getTo().c_str(),
                            message->getPlainTextRepresentation());

            logger().log(opencog::Logger::DEBUG,
                         "RouterServerSocket - Message sent to %s using HttpRequest.", id.c_str());

            delete(message);
            if (limit > 0) limit--;
        }
    } else {
        std::string cmd;
        if (!master->dataSocketConnection(id)) {
            return;
        }
        int sock = master->getDataSocket(id);

        while (limit != 0 && !master->getMessageCentral()->isQueueEmpty(id)) {

            // Remember that all message stored within the router are of
            // RouterMessage type.
            RouterMessage *message = (RouterMessage *)master->getMessageCentral()->pop(id);
            sprintf(s, "cSTART_MESSAGE %s %s %d", message->getFrom().c_str(),
                    message->getTo().c_str(), message->getEncapsulateType());
            logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Sending message (socket = %d): <%s>", sock, s);

            cmd.assign(s);
            cmd.append("\n");
            unsigned int sentBytes = 0;
            if ( ( sentBytes = send(sock, cmd.c_str(), cmd.length(), 0) ) != cmd.length() ) {
                logger().log(opencog::Logger::ERROR, "RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, cmd.length() );
                master->closeDataSocket(id);
                master->closeControlSocket(id);
                master->markElementUnavailable(id);
                delete(message); // TODO: Shouldn't message be put back to the queue?
                return;
            }

            std::istringstream stream(message->getPlainTextRepresentation());
            std::string line;
            while (getline(stream, line)) {
                logger().log(opencog::Logger::DEBUG, "Sending line <d%s>", line.c_str());
                line.insert(0, "d");
                line.append("\n");
                sentBytes = 0;
                if ( ( sentBytes = send(sock, line.c_str(), line.length(), 0) ) != line.length() ) {
                    logger().log(opencog::Logger::ERROR, "RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, line.length() );
                    master->closeDataSocket(id);
                    master->closeControlSocket(id);
                    master->markElementUnavailable(id);
                    delete(message); // TODO: Shouldn't message be put back to the queue?
                    return;
                }
                // TODO: Shouldn't the protocol be changed to expect a feedback (OK or FAILED) per message here?
                // And, if OK is not received, message may be kept on the queue (until it be eventually sent or,
                // at least, until N attempts are made)
            }
            delete(message);
            if (limit > 0) limit--;
        }
        unsigned int sentBytes = 0;
        cmd.assign("cNO_MORE_MESSAGES\n");
        if ( ( sentBytes = send(sock, cmd.c_str(), cmd.length(), 0) ) != cmd.length() ) {
            logger().log(opencog::Logger::ERROR, "RouterServerSocket -  Mismatch in number of sent bytes. %d was sent, but should be %d", sentBytes, cmd.length() );
            master->closeControlSocket(id);
            master->closeDataSocket(id);
            master->markElementUnavailable(id);
            return;
        }


        if (!master->noAckMessages) {
            logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Waiting OK (after sending message).");
#define BUFFER_SIZE 256
            char response[BUFFER_SIZE];
            int receivedBytes = 0;
            if ( (receivedBytes = recv(sock, response, BUFFER_SIZE - 1, 0 ) ) <= 0 ) {
                logger().log(opencog::Logger::ERROR, "RouterServerSocket - Invalid response. recv returned %d ", receivedBytes );
                master->closeDataSocket(id);
                master->closeControlSocket(id);
                master->markElementUnavailable(id);
                return;
            }

            response[receivedBytes] = '\0'; // Assure null terminated string
            // chomp all trailing slashes from string
            int i;
            for ( i = receivedBytes - 1; i >= 0 && (response[i] == '\n' || response[i] == '\r')  ; --i ) {
                response[i] = '\0';
            }

            logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Received response (after chomp): '%s' bytes: %d",
                         response, receivedBytes );
            std::string answer = response;

            if (answer == NetworkElement::OK_MESSAGE) {
                logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Sucessfully sent messages to '%s'.",
                             id.c_str());
            } else {
                logger().log(opencog::Logger::ERROR, "RouterServerSocket - Failed to send messages to '%s'. (answer = %s)",
                             id.c_str(), answer.c_str());
            }
        }

        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - id <%s> message queue size %d", id.c_str(), master->getMessageCentral()->queueSize(id));
    }

    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - OK");
}

bool RouterServerSocket::sendHttpRequest(const std::string& messageFrom, const std::string& messageTo, const std::string& messageText)
{
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Send HttpRequest. From '%s'. To '%s'.", messageFrom.c_str(), messageTo.c_str());
    // example of url: "http://localhost:8211/petproxy/pet/78/action/5dfe52f9-7344-4ffe-99b7-493f428d5b34"
    //
    std::string url = "http://";
    url += master->getIPAddress(currentMessageTo).c_str();
    url += ":" ;
    url += opencog::toString(master->getPortNumber(currentMessageTo));
    url += "/petproxy/pet/";
    url += messageFrom;
#if 1
    url += "/action";
#else
    // TODO: Remove this uggly hack when the action plan id is not needed in the URI anymore...
    char* petId = strdup(messageFrom.c_str());
    char* p = petId;
    while (*p != ':' && *p != '\0') p++;
    *p = '\0';
    char* actionPlanId = p + 1;
    url += petId;
    url += "/action/";
    url += actionPlanId;
    free(petId);
#endif
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - HTTP request to url = %s", url.c_str());

    StdoutLog log;
    SocketHandler h(&log);
    RouterHttpPostSocket sock( h, url);
    sock.SetBody(messageText);
    sock.Open();
    h.Add(&sock);
    //sock.AddFile("actionPlan", ACTION_PLAN_TEST_XML_FILE, "text/plain"); // Old method using HttpPostSocket
    h.Select(1, 0);
    while (h.GetCount()) {
        h.Select(1, 0);
    }
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Status: %s %s", sock.GetStatus().c_str(), sock.GetStatusText().c_str());
    return (!strcmp(sock.GetStatus().c_str(), "200"));
}

void RouterServerSocket::storeNewMessage()
{

    std::string answer;
    bool addToMasterQueue = true;
    bool notifyMessageArrival = true;

    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - New message arrived. From '%s'. To '%s'. Type '%d'. Lines: %d", currentMessageFrom.c_str(), currentMessageTo.c_str(), currentMessageType, currentMessageSize);
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Message body:\n%s", currentMessageText.c_str());

    bool knownFrom = master->knownID(currentMessageFrom);
    bool knownTo   = master->knownID(currentMessageTo);

    if (!knownFrom) {
        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Unknown from '%s'. Discarding message.", currentMessageFrom.c_str());
        answer.assign("FAILED - Unknown <from> ID: "); answer.append(currentMessageFrom);
        sendAnswer(answer);
        return;

    } else if (!knownTo) {
        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Unknown to '%s' (from '%s'). Wont be notified.", currentMessageTo.c_str(), currentMessageFrom.c_str());

        // since the component is not registred there is no need to notify it about new messages
        notifyMessageArrival = false;

        // Marks this element as unavailable so that Router notifies all other elements
        // that eventually has this element as available.
        master->markElementUnavailable(currentMessageTo);

        //answer.assign("FAILED - Unknown <to> ID: "); answer.append(currentMessageTo);
        //sendAnswer(answer);
        //return;
    } else if (!master->isElementAvailable(currentMessageTo)) {
        logger().log(opencog::Logger::WARN, "RouterServerSocket - Element '%s' unavailable. Discarding message.", currentMessageTo.c_str());
        answer.assign("FAILED - Element unavailable: "); answer.append(currentMessageTo);
        sendAnswer(answer);
        return;
    }

    if (currentMessageTo == opencog::config().get("PROXY_ID") &&
            currentMessageFrom != opencog::config().get("SPAWNER_ID") &&
            master->getPortNumber(currentMessageTo) == 8211) { // TODO: this is a temporary hack so that we can use both proxy and mocky proxy
        // Send message as a http request
        addToMasterQueue = !sendHttpRequest(currentMessageFrom, currentMessageTo, currentMessageText);
        notifyMessageArrival = false;
    }

    if (addToMasterQueue) {

//        try {

        // Router has its own message type (RouterMessage) to encapsulate all
        // messages received. It is not Router's duty to check if a message
        // is well formed or not. This sould be done by the NetworkElement
        // receiving the message.



        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Building Message object.");
        Message *message = Message::routerMessageFactory(currentMessageFrom, currentMessageTo,
                           currentMessageType, currentMessageText);

        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Queueing message.");
        master->getMessageCentral()->push(currentMessageTo, message);

//        } catch (InvalidParamException& e){
//         logger().log(opencog::Logger::ERROR,
//                "RouterServerSocket - Discarding message with invalid parameter.\nMessage:\n%s.",
//                currentMessageText.c_str());
//        }
    }

    // Ack sender that message was delivered successfully
    answer.assign(NetworkElement::OK_MESSAGE);
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Sending OK.");
    sendAnswer(answer);

    if (notifyMessageArrival) {
        if (no_msg_arrival_notification) {
            sendRequestedMessages(currentMessageTo, -1);
        } else {
            master->notifyMessageArrival(currentMessageTo, 1);
        }
    }
    logger().log(opencog::Logger::FINE, "RouterServerSocket - Finished queueing messages.");
}


void RouterServerSocket::OnLine(const std::string& line)
{

    logger().log(opencog::Logger::DEBUG, "RouterServerSocket[%p] - Received line <%s>",
                 this, line.c_str());
    logger().log(opencog::Logger::DEBUG, "RouterServerSocket - State = %d", currentState);

    switch (currentState) {
    case WAITING_COMMAND: {
        std::string command;
        std::queue<std::string> args;
        NetworkElement::parseCommandLine(line, command, args);
        logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Parsed command <%s>. Args size: %d.", command.c_str(), args.size());
        if (command == "LOGIN") {
            // handshake
            std::string id = args.front();
            args.pop();
            std::string ip = args.front();
            args.pop();
            int port = atoi(args.front().c_str());
            args.pop();

            logger().log(opencog::Logger::INFO,
                         "RouterServerSocket - Handshaking: id = %s ip = %s port = %d.",
                         id.c_str(), ip.c_str(), port);

            addNetworkElement(id, ip, port);

        } else if (command == "LOGOUT") {
            // logout an NetworkElement - must of the time an OPC
            std::string id = args.front();
            args.pop();

            logger().log(opencog::Logger::INFO,
                         "RouterServerSocket - Logout: id = %s.", id.c_str());

            master->removeNetworkElement(id);
            sendAnswer(NetworkElement::OK_MESSAGE);

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

            logger().log(opencog::Logger::INFO,
                         "RouterServerSocket - Clear message queue: id = '%s' requesting for '%s'.",
                         requestorId.c_str(), targetId.c_str());

            master->clearNetworkElementMessageQueue(targetId);
            sendAnswer(NetworkElement::OK_MESSAGE);

        } else if (command == "SHUTDOWN") {
            // exit
            logger().log(opencog::Logger::INFO, "RouterServerSocket - Shutdown router.");
            master->shutdown();

        } else if (command == "REQUEST_UNREAD_MESSAGES") {
            // request messages
            std::string id = args.front();
            args.pop();
            int limit = atoi(args.front().c_str());
            args.pop();
            logger().log(opencog::Logger::INFO, "RouterServerSocket - %s requested messages (limit = %d).", id.c_str(), limit);
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
            logger().log(opencog::Logger::INFO, "RouterServerSocket - Message arriving. From '%s' To '%s'.", currentMessageFrom.c_str(), currentMessageTo.c_str());
        } else {
            logger().log(opencog::Logger::ERROR, "RouterServerSocket - Invalid command (%s) received. Discarding it.", command.c_str());
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
            logger().log(opencog::Logger::DEBUG, "RouterServerSocket - Message complete.");
            storeNewMessage();
            currentState = WAITING_COMMAND;
        }
        break;
    }
    default: {
        logger().log(opencog::Logger::WARN, "RouterServerSocket - Invalid state (%d). Comming back to WAITING_COMMAND (discarded last line).", currentState);
        currentState = WAITING_COMMAND;
        break;
    }
    }
}

