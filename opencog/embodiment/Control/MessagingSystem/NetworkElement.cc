/*
 * opencog/embodiment/Control/MessagingSystem/NetworkElement.cc
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


#include <iostream>
#include <sstream>
#include <vector>
#include <signal.h>

//for the random generator initilization
#include <cmath>
#include <ctime>

#ifndef WIN32
#include <unistd.h>
#endif

#include <boost/regex.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

#include <opencog/util/foreach.h>
#include <opencog/embodiment/Control/LoggerFactory.h>

#include "Message.h"
#include "MessageFactory.h"
#include "NetworkElement.h"
#include "NetworkElementCommon.h"
#include "ServerSocket.h"

using namespace opencog::messaging;
using namespace opencog;
using boost::asio::ip::tcp;

unsigned sleep(unsigned seconds);

// static definition
bool NetworkElement::stopListenerThreadFlag = false;
bool NetworkElement::socketListenerIsReady = false;
std::vector<ServerSocket*> NetworkElement::serverSockets;

// Public interface

NetworkElement::~NetworkElement()
{
    logger().debug("NetworkElement - Destructor called.");

    if (!NetworkElement::stopListenerThreadFlag) {
        NetworkElement::stopListenerThread();
    }

    if (this->sock != NULL) {
        this->sock->close();
        delete this->sock;
    }

    foreach(ServerSocket* ss, serverSockets) {
        delete ss;
    }

    pthread_mutex_destroy(&messageQueueLock);
    pthread_mutex_destroy(&tickLock);
    pthread_mutex_destroy(&socketAccessLock);
}


NetworkElement::NetworkElement()
{
    this->sock = NULL;
    this->subclass_initialized = false;
    pthread_mutex_init(&socketAccessLock, NULL);
    pthread_mutex_init(&tickLock, NULL);
    pthread_mutex_init(&messageQueueLock, NULL);
}

NetworkElement::NetworkElement(const std::string &myId, const std::string &ip, int portNumber)
{
    this->sock = NULL;
    this->subclass_initialized = false;
    initialize(myId, ip, portNumber);
}

void NetworkElement::initialize(const std::string &myId, const std::string &ip, int portNumber)
{
    this->portNumber = portNumber;
    this->myId.assign(myId);
    this->ipAddress.assign(ip);

    // Initializes the main logger (static logger for this process)
    // merely specify the file name of the logger
    opencog::logger() = opencog::control::LoggerFactory::getLogger(myId);

    this->routerID.assign(opencog::config().get("ROUTER_ID"));
    this->routerIP.assign(opencog::config().get("ROUTER_IP"));
    this->routerPort = opencog::config().get_int("ROUTER_PORT");
    this->noAckMessages = config().get_bool("NO_ACK_MESSAGES");

    logger().info("NetworkElement - Router address %s:%d", routerIP.c_str(), routerPort);

    //messageCentral has been initialized
    this->messageCentral.createQueue(this->myId);
    this->messageCentral.createQueue("ticks_queue");
    pthread_mutex_init(&messageQueueLock, NULL);
    pthread_mutex_init(&tickLock, NULL);
    pthread_mutex_init(&socketAccessLock, NULL);

    stopListenerThreadFlag = false;
    numberOfUnreadMessages = 0;
    tickNumber = 0;
    ServerSocket::setMaster(this);
    if (!startListener()) {
        logger().error("NetworkElement - Could not initialize listener or its thread");
        exit(-1);
    }
    handshakeWithRouter();
}

void NetworkElement::markAsInitialized()
{
    subclass_initialized = true;
}

bool NetworkElement::isInitialized()
{
    return subclass_initialized;
}

bool NetworkElement::haveUnreadMessage()
{
    bool returnValue;
    pthread_mutex_lock(&messageQueueLock);
    returnValue = (numberOfUnreadMessages > 0);
    //logger().debug("NetworkElement::haveUnreadMessage() - Number of unread messages: %d", numberOfUnreadMessages);
    pthread_mutex_unlock(&messageQueueLock);
    return returnValue;
}

bool NetworkElement::retrieveMessages(int limit)
{

    if (! haveUnreadMessage()) {
        return false;
    }

    //logger().debug("NetworkElement - Main thread: request message retrieving.");
    requestUnreadMessages(limit);
    return true;
}

bool NetworkElement::sendMessage(Message &msg)
{

    logger().debug("NetworkElement - sendMessage - init");
    std::string messagePayload = msg.getPlainTextRepresentation();

    if (logger().isFineEnabled()) {
        logger().fine("NeworkElement - Preparing to send message:\n%s",
                      messagePayload.c_str() );
        logger().fine("NetworkElement - Sending message to %s.",
                      msg.getTo().c_str() );
    } // if

    int lineCount = 1;
    unsigned int i;
    for ( i = 0; i < messagePayload.length(); ++i ) {
        if ( messagePayload[i] == '\n') {
            ++lineCount;
        } // if
    } // for

    std::stringstream message;
    message << "NEW_MESSAGE ";
    message << msg.getFrom();
    message << " ";
    message << msg.getTo();
    message << " ";
    message << msg.getType();
    message << " ";
    message << lineCount << "\n";
    message << messagePayload << "\n";

    if (logger().isFineEnabled()) {
        logger().fine("NetworkElement - Sending command <%s> (%d lines)",
                      message.str().c_str(), lineCount);
    }

    std::string msg_to_send = message.str();
    std::string response = sendMessageToRouter( msg_to_send );

    if ( response == NetworkElementCommon::OK_MESSAGE ) {
        logger().debug("NetworkElement - sendMessage - response = 'OK'");
        return true;
    } else {
        logger().error("NetworkElement - sendMessage - response = %s. Expected OK.",
                       response.c_str());
        return false;
    } 
}

int NetworkElement::getPortNumber()
{
    return portNumber;
}

const std::string &NetworkElement::getIPAddress()
{
    return ipAddress;;
}

const std::string &NetworkElement::getID()
{
    return myId;
}

// Private interface

// Call-back from client/server sockets

void NetworkElement::newMessageInRouter(int numMessages)
{

    logger().debug("NetworkElement - Notified about new message in router.");
    pthread_mutex_lock(&messageQueueLock);
    numberOfUnreadMessages += numMessages;
    pthread_mutex_unlock(&messageQueueLock);
    logger().debug("NetworkElement::newMessageinRouter() - Number of unread messages: %d", numberOfUnreadMessages);
}

void NetworkElement::newMessageRead(const std::string &from, const std::string &to, int type, const std::string &msg)
{


    logger().debug("NetworkElement - newMessageRead");
    if (logger().isFineEnabled())
        logger().fine("NetworkElement - Received message. From '%s'. To '%s', Type '%d'.\n%s",
                     from.c_str(), to.c_str(), type, msg.c_str());

    try {

        Message *message = messageFactory(from, to, type, msg);
        this->messageCentral.push(this->myId, message);

    } catch (opencog::InvalidParamException& e) {
        logger().error(
                     "NetworkElement - Discarding message with invalid parameter.\nMessage:\n%s.",
                     msg.c_str());
    }

    pthread_mutex_lock(&messageQueueLock);
    numberOfUnreadMessages--;
    pthread_mutex_unlock(&messageQueueLock);
    logger().debug("NetworkElement::newMessageRead() - Number of unread messages: %d", numberOfUnreadMessages);
}

void NetworkElement::noMoreMessages()
{

    logger().debug("NetworkElement - noMoreMessages.");
    //pthread_cond_broadcast(&mainThreadBed);
}

void NetworkElement::unavailableElement(const std::string &id)
{
    logger().debug("NetworkElement - unavailableElement.");
    // Handles special case where Router sends unavailable notification about itself,
    // which means it asks for handshake in a new connection
    if (id == this->routerID) {
        if (this->sock != NULL) {
            this->sock->close();
            delete this->sock;
            this->sock = NULL;
        }
        handshakeWithRouter();
    } else {
        markAsUnavailableElement(id);
    }
}

void NetworkElement::availableElement(const std::string &id)
{
    markAsAvailableElement(id);
}

// general

bool NetworkElement::startListener()
{
    pthread_attr_init(&socketListenerAttr);
    pthread_attr_setscope(&socketListenerAttr, PTHREAD_SCOPE_PROCESS);
    pthread_attr_setinheritsched(&socketListenerAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&socketListenerAttr, PTHREAD_CREATE_DETACHED);
    int errorCode = pthread_create(&socketListenerThread, &socketListenerAttr, NetworkElement::portListener, &portNumber);
    if (errorCode) {
        NetworkElementCommon::reportThreadError(errorCode);
        return false;
    } else {
        // Give time so that it starts listening server port before sending any message to ROUTER.
        int count = config().get_int("WAIT_LISTENER_READY_TIMEOUT");
        do {
            logger().debug("NetworkElement - startListener(): waiting for listener to be ready.");
            sleep(1);
            if (--count <= 0) {
                break;
            }
        } while (!socketListenerIsReady);
        return socketListenerIsReady;
    }
}

bool NetworkElement::sendCommandToRouter(const std::string &cmd)
{

    std::string response = sendMessageToRouter( cmd );

    if ( response == NetworkElementCommon::OK_MESSAGE ) {
        logger().debug("NetworkElement - response = 'OK'");
        return true;
    } else if ( response == NetworkElementCommon::FAILED_TO_CONNECT_MESSAGE || response == NetworkElementCommon::FAILED_MESSAGE ) {
        logger().warn(
                     "NetworkElement - Unable to connect to Router in sendCommandToRouter.");
        // cannot connect to router, add it to unavailable elements list
        markAsUnavailableElement(this->routerID);
        return false;
    } else {
        logger().error(
                     "NetworkElement - SendCommandToRouter response = %s. Expected OK.", response.c_str());
        return false;
    } // else
}

void NetworkElement::requestUnreadMessages(int limit)
{

    logger().debug("NetworkElement - Requesting unread messages (limit = %d).", limit);
    char s[256];
    sprintf(s, "REQUEST_UNREAD_MESSAGES %s %d", myId.c_str(), limit);
    std::string cmd = s;
    sendCommandToRouter(cmd);
}

void NetworkElement::handshakeWithRouter()
{

    char s[256];
    sprintf(s, "LOGIN %s %s %d", myId.c_str(), ipAddress.c_str(), portNumber);
    std::string cmd = s;
    logger().debug("NetworkElement - Handshaking with router...");
    if (sendCommandToRouter(cmd)) {
        logger().info("NetworkElement - Handshake with router done.");
    } else {
        logger().error("NetworkElement - Handshake with router failed.");
    }
}

void NetworkElement::logoutFromRouter()
{

    char s[256];
    sprintf(s, "LOGOUT %s", myId.c_str());
    std::string cmd = s;
    logger().debug("NetworkElement - Logging out from router...");
    if (sendCommandToRouter(cmd)) {
        logger().info("NetworkElement - Logout from router done.");
    } else {
        logger().error("NetworkElement - Logout from router failed.");
    }
}

// static methods

void *NetworkElement::portListener(void *arg)
{

    logger().debug("NetworkElement - Port listener executing.");

    int port = *((int*) arg);

    logger().info("NetworkElement - Binding to port %d.", port);

    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));
        logger().debug("NetworkElment - Listening to port %d.", port);
        socketListenerIsReady = true;
        while (!stopListenerThreadFlag)
        {
            ServerSocket* ss = new ServerSocket();
            serverSockets.push_back(ss);
            acceptor.accept(ss->getSocket());
            ss->start();
        }
    } catch (boost::system::system_error& e)
    {
        logger().error("NetworkElement - Boost system error listening to port %d. Error message: %s", port, e.what());
    } catch (std::exception& e)
    {
        logger().error("NetworkElement - Error listening to port %d. Exception message: %s", port, e.what());
    }

    logger().debug("Port listener finished.");
    return NULL;
}

void NetworkElement::markAsUnavailableElement(const std::string &id)
{
    logger().info("NetworkElement - Adding %s to unavailable list.",
                 id.c_str());

    unavailableElements.insert(id);

    // if the unavailable element is the router then all unread message counters
    // should be reset.
    if (id == this->routerID) {
        pthread_mutex_lock(&messageQueueLock);
        numberOfUnreadMessages = 0;
        pthread_mutex_unlock(&messageQueueLock);
    }
}

void NetworkElement::markAsAvailableElement(const std::string &id)
{

    if (!isElementAvailable(id)) {

        // remove the element from the list
        unavailableElements.erase(id);
        logger().info(
                     "NetworkElement - Removing %s from unavailable list.", id.c_str());

        if (id == this->routerID) {
            handshakeWithRouter();
        }
    }
}

bool NetworkElement::isElementAvailable(const std::string &id)
{
    return (unavailableElements.find(id) == unavailableElements.end());
}


void NetworkElement::stopListenerThread()
{
    NetworkElement::stopListenerThreadFlag = true;
    // wait for thread to return
    pthread_join(socketListenerThread, NULL);
}

bool NetworkElement::connectToRouter(void)
{
    if ( this->sock != NULL ) {
        logger().debug(
                     "NetworkElement - connectToRouter. Connection already established" );
        return true;
    } // if

    sock = NULL;
    try {
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), this->routerIP, boost::lexical_cast<std::string>(this->routerPort));
        tcp::resolver::iterator iterator = resolver.resolve(query);
        sock = new tcp::socket(io_service);
        logger().debug("NetworkElement - connectToRouter: created new socket: %p.", sock);
        sock->connect(*iterator);
        logger().debug(
            "NetworkElement - connectToRouter: Connection established. ip=%s, port=%d",
            this->routerIP.c_str(), this->routerPort);
        return true;
    }catch(std::exception& e){
        logger().error(
            "NetworkElement - connectToRouter: Unable to connect to router. ip=%s, port=%d. Exception Message: %s",
            this->routerIP.c_str(), this->routerPort, e.what());
        if (sock != NULL) delete sock;
            sock = NULL;
    }
    markAsUnavailableElement(this->routerID);
    return false;
}

std::string NetworkElement::sendMessageToRouter( const std::string& message )
{
    pthread_mutex_lock(&socketAccessLock);

    if ( message.length( ) == 0 ) {
        logger().error("NetworkElement - sendMessageToRouter. Invalid zero length message" );
        pthread_mutex_unlock(&socketAccessLock);
        return NetworkElementCommon::FAILED_MESSAGE;
    } // if

    if ( !connectToRouter( ) ) {
        pthread_mutex_unlock(&socketAccessLock);
        return NetworkElementCommon::FAILED_MESSAGE;
    } // if

    std::string sentText =  message;
    if ( sentText[sentText.length() -1] != '\n' ) {
        sentText += "\n";
    } // if

    boost::system::error_code error;
    boost::asio::write(*sock, boost::asio::buffer(sentText),
                       boost::asio::transfer_all(), error);
    if (error) {
        logger().debug("NetworkElement - sendMessageToRouter (socket = %p) Error transfering data.", sock);
        return NetworkElementCommon::FAILED_MESSAGE;
    }

    if (noAckMessages) {
        pthread_mutex_unlock(&socketAccessLock);
        return NetworkElementCommon::OK_MESSAGE;
    }

#define BUFFER_SIZE 256
    char buffer[BUFFER_SIZE];
    size_t receivedBytes = sock->read_some(boost::asio::buffer(buffer), error);
    if (error && error != boost::asio::error::eof) {
        logger().error("NetworkElement - sendMessageToRouter. Error receiving data. %d bytes received", receivedBytes );
        this->sock->close();
        this->sock = NULL;
        pthread_mutex_unlock(&socketAccessLock);
        return NetworkElementCommon::FAILED_MESSAGE;
    } // if

    buffer[receivedBytes] = '\0'; // Assure null terminated string

    // chomp all trailing slashes from string
    int i;
    for ( i = receivedBytes - 1; i >= 0 && (buffer[i] == '\n' || buffer[i] == '\r'); --i ) {
        buffer[i] = '\0';
    } // if

    logger().debug("NetworkElement - sendMessageToRouter. Received response (after chomp): '%s' bytes: %d", buffer, receivedBytes );

    pthread_mutex_unlock(&socketAccessLock);
    return buffer;
}

unsigned int NetworkElement::getIncomingQueueSize()
{
    return messageCentral.queueSize(myId);
}

bool NetworkElement::isIncomingQueueEmpty()
{
    return messageCentral.isQueueEmpty(myId);
}

Message* NetworkElement::popIncomingQueue()
{
    return messageCentral.pop(myId);
}

