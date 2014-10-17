/*
 * opencog/embodiment/Control/MessagingSystem/Router.cc
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


#include "Router.h"

#include <opencog/util/foreach.h>

#include <opencog/embodiment/Control/LoggerFactory.h>
#include <fstream>

#include <opencog/util/files.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/oc_assert.h>

#include "RouterServerSocket.h"
#include "NetworkElementCommon.h"

#include <boost/lexical_cast.hpp>

using namespace opencog::messaging;
using namespace opencog;
using boost::asio::ip::tcp;

std::vector<RouterServerSocket*> Router::serverSockets;
bool Router::stopListenerThreadFlag = false;

Router::Router()
{

    running = true;
    // This will force the router to send AVAILABLE_ELEMENT for all
    // known NE as soon it starts up...
    lastNotifyTimestamp = 0;
    messageCentral = new MemoryMessageCentral();

    // This a way to make router fail and raise exception for testing
    //exception = false;

    RouterServerSocket::setMaster(this);

    routerId = config().get("ROUTER_ID");
    routerPort = config().get_int("ROUTER_PORT");
    routerAvailableNotificationInterval = 
        config().get_int("ROUTER_AVAILABLE_NOTIFICATION_INTERVAL");
    noAckMessages = config().get_bool("NO_ACK_MESSAGES");

    logger() = opencog::control::LoggerFactory::getLogger(routerId);

    pthread_mutex_init(&unavailableIdsLock, NULL);
    stopListenerThreadFlag = false;
}

Router::~Router()
{
    if (!Router::stopListenerThreadFlag) {
        Router::stopListenerThread();
    }

    for(RouterServerSocket* rss : serverSockets) {
        delete rss;
    }

    delete(messageCentral);
}

void Router::startListener()
{
    pthread_attr_init(&socketListenerAttr);
    pthread_attr_setscope(&socketListenerAttr, PTHREAD_SCOPE_PROCESS);
    pthread_attr_setinheritsched(&socketListenerAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&socketListenerAttr, PTHREAD_CREATE_DETACHED);
    int errorCode = pthread_create(&socketListenerThread, &socketListenerAttr, Router::portListener, &routerPort);
    if (errorCode) {
        NetworkElementCommon::reportThreadError(errorCode);
    } else {
        sleep(1); // Give time so that it starts listening server port before sending any message to NE's.
    }
}

void *Router::portListener(void *arg)
{
    logger().debug("Router - Port listener executing.");

    int port = *((int*) arg);

    logger().info("Router - Binding to port %d.", port);

    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));
        logger().debug("Router - Listening to port %d.", port);
        while (!stopListenerThreadFlag)
        {
            RouterServerSocket* rss = new RouterServerSocket();
            serverSockets.push_back(rss);
            acceptor.accept(rss->getSocket());
            rss->start();
        }
    } catch (boost::system::system_error& e)
    {
        logger().error("Router - Boost system error listening to port %d. Error message: %s", port, e.what());
    } catch (std::exception& e)
    {
        logger().error("Router - Error listening to port %d. Exception message: %s", port, e.what());
    }

    logger().debug("Port listener finished.");
    return NULL;
}

void Router::stopListenerThread()
{
    Router::stopListenerThreadFlag = true;
    // wait for thread to return
    pthread_join(socketListenerThread, NULL);
}

void Router::run()
{

    /**
     * Recovery operation (if needed). First mount the filename full path and
     * then check if such file exists in filesystem. If so, start recovery
     * process.
     */
    std::string recoveryFile = config().get("ROUTER_DATABASE_DIR");
    expandPath(recoveryFile);
    recoveryFile.append("/");
    recoveryFile.append(config().get("ROUTER_DATA_FILE"));

    if (fileExists(recoveryFile.c_str())) {
        recoveryFromPersistedData(recoveryFile);
        remove(recoveryFile.c_str());
        startListener();
        notifyElementAvailability(routerId, false);
    } else {
        startListener();
    }


    do {
        // sleeps a bit since the only task to do here is availability notification to Network elements.
        // So, no need to be busy waiting all the time
        usleep(50000);

        // periodically router notification
        time_t now = time(NULL);

        if (now - lastNotifyTimestamp > routerAvailableNotificationInterval) {
            notifyElementAvailability(routerId, true);
            lastNotifyTimestamp = now;
        }

        // copy set of elements whose availability notification is to be sent to a local variable
        pthread_mutex_lock(&unavailableIdsLock);
        std::set<std::string> localToNotifyAvailability(toNotifyAvailability);
        toNotifyAvailability.clear();
        pthread_mutex_unlock(&unavailableIdsLock);

        //logger().debug("Router - Checked pending availability notifications: %d", localToNotifyAvailability.size());

        // Now sends the available element notifications
        for (const std::string& id : localToNotifyAvailability)
            notifyElementAvailability(id, true);

        // copy set of elements whose unavailability notification is to be sent to a local variable
        pthread_mutex_lock(&unavailableIdsLock);
        std::set<std::string> localToNotifyUnavailability(toNotifyUnavailability);
        toNotifyUnavailability.clear();
        pthread_mutex_unlock(&unavailableIdsLock);

        //logger().debug("Router - Checked pending unavailability notifications: %d",
        //  localToNotifyUnavailability.size());

        // Now sends the unavailable element notifications
        for (const std::string& id : localToNotifyUnavailability)
            notifyElementAvailability(id, false);

        // NOTE: Uncomment the code to test a way to make router fail and raise
        // an execption
        //if (exception) {
        //    throw NetworkException(TRACE_INFO, "Router - Generating an exception.");
        //}

    } while (running);
}

void Router::shutdown()
{
    running = false;

    // This a way to make router fail and raise a NetworkException for testing
    //exception = true;
}

// call-back interface

bool Router::knownID(const std::string &id)
{
    std::map<std::string, int>::iterator iter1 = portNumber.find(id);
    if (iter1 == portNumber.end()) {
        // component not registered in router yet ... create it's message queue
        // since the component may be starting up.
        if (!messageCentral->existsQueue(id)) {
            messageCentral->createQueue(id);
        }
        return false;
    } else {
        return true;
    }
}

MessageCentral* Router::getMessageCentral()
{
    return messageCentral;
}

int Router::getPortNumber(const std::string &id) const
{
    auto it = portNumber.find(id);
    if (it != portNumber.end())
        return it->second;
    OC_ASSERT(false, "Router::getPortNumber - "
              "the network element %s is unknown", id.c_str());
    return 0;
}

const std::string &Router::getIPAddress(const std::string &id) const
{
    auto it = ipAddress.find(id);
    if (it != ipAddress.end())
        return it->second;
    OC_ASSERT(false, "Router::getIPAddress - "
              "The network element %s is unknown", id.c_str());
    return dummyIP;
}

tcp::socket* Router::getControlSocket(const std::string &id) const
{
    tcp::socket* sock = NULL;
    Id2SocketMap::const_iterator it = controlSockets.find(id);
    if (it != controlSockets.end()) {
        sock = it->second;
    } else {
        OC_ASSERT(false, "Router::getControlSocket - "
                  "The network element %s is unknown", id.c_str());
    }
    //logger().debug("Getting control socket(%d) for element '%s'.", sock,  id.c_str());
    return sock;
}

tcp::socket* Router::getDataSocket(const std::string &id) const
{
    tcp::socket* sock = NULL;
    Id2SocketMap::const_iterator it = dataSockets.find(id);
    if (it != dataSockets.end()) {
        sock = it->second;
    } else {
        OC_ASSERT(false, "Router::getDataSocket - "
                  "The network element %s is unknown", id.c_str());
    }
    //logger().debug("Getting data socket(%d) for element '%s'.", sock,  id.c_str());
    return sock;
}

void Router::closeControlSocket(const std::string &id)
{
    logger().debug("Closing control socket for element '%s'.", id.c_str());
    Id2SocketMap::iterator it = controlSockets.find(id);
    if (it != controlSockets.end()) {
        tcp::socket* sock = it->second;
        sock->close();
        delete sock;
        controlSockets.erase(it);
        logger().debug("Closed control socket for element '%s'.", id.c_str());
    }
}

void Router::closeDataSocket(const std::string &id)
{
    logger().debug("Closing data socket for element '%s'.", id.c_str());
    Id2SocketMap::iterator it = dataSockets.find(id);
    if (it != dataSockets.end()) {
        tcp::socket* sock = it->second;
        sock->close();
        delete sock;
        dataSockets.erase(it);
        logger().debug("Closed data socket for element '%s'.", id.c_str());
    }
}

int Router::addNetworkElement(const std::string &strId, const std::string &strIp, int port)
{

    std::string id(strId);
    std::string ip(strIp);
    int errorCode = NO_ERROR;

    ipAddress[id] = ip;
    portNumber[id] = port;
    logger().debug("Router - Adding component: '%s' - IP: '%s', Port: '%d'.",
                   id.c_str(), ipAddress[id].c_str(), port);

    // new data inserted, updates persisted router table
    try {
        persistState();
    } catch (IOException& e) { }

    // erase all messages from queue if have any
    if (id == config().get("LS_ID") ||
        id == config().get("SPAWNER_ID")) {

        messageCentral->createQueue(id, true);
        errorCode = NO_ERROR;

        // keep the messages within the queue, if any
    } else {

        if (!messageCentral->existsQueue(id)) {
            messageCentral->createQueue(id);
            errorCode = NO_ERROR;
        } else {
            if (!messageCentral->isQueueEmpty(id)) {
                errorCode = HAS_PENDING_MSGS;
            }
        }
    }

    // If the element has correctly executed the handshaking protocol and is
    // marked as an unavailable element (due to some previous crash or somethig
    // else), then the mark should be removed allowing others to communicate with
    // the component.
    if ((errorCode == NO_ERROR || errorCode == HAS_PENDING_MSGS) && !isElementAvailable(id)) {
        markElementAvailable(id);
    }

    return errorCode;
}

void Router::removeNetworkElement(const std::string &id)
{
    ipAddress.erase(id);
    portNumber.erase(id);
    closeSockets(id);

    messageCentral->removeQueue(id);

    // save modifications into persisted data file
    try {
        persistState();
    } catch (IOException& e) { }
}

void Router::clearNetworkElementMessageQueue(const std::string &id)
{
    messageCentral->clearQueue(id);
}

void Router::persistState() const
{
    // TODO: add some timestamp info to filename
    std::string path = config().get("ROUTER_DATABASE_DIR");
    expandPath(path);

    if (!createDirectory(path.c_str())) {
        logger().error("Router - Cannot create directory '%s'.",
                     path.c_str());
        return;
    }

    // TODO: Insert timestamp information into routerInfo file
    std::string filename = path + "/" + config().get("ROUTER_DATA_FILE");
    remove(filename.c_str());

    std::ofstream routerFile(filename.c_str());
    routerFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);

    try {

        for (const auto& IdIp : ipAddress) {

            std::string id = IdIp.first;
            std::string ip = IdIp.second;
            std::string port = toString(getPortNumber(id));

            // save data
            routerFile << id << " " << ip << " " << port  << std::endl;
        }
    } catch (std::ofstream::failure e) {
        routerFile.close();
        throw IOException(TRACE_INFO, "Router - Unable to save Router information.");
    }

    routerFile.close();
}

void Router::recoveryFromPersistedData(const std::string& fileName)
{

    char line[256];
    std::ifstream fin(fileName.c_str());

    std::string id;
    std::string ip;
    int port;

    while (!fin.eof()) {
        fin.getline(line, 256);

        // not a comment or an empty line
        if (line[0] != '#' && line[0] != 0x00) {
            std::istringstream in ((std::string)line);

            in >> id;
            in >> ip;
            in >> port;

            // reconstruct router table
            ipAddress[id] = ip;
            portNumber[id] = port;
            // since Router does not know about NE availability, mark them as unavailable
            // until they handshake again.
            unavailableIds.insert(id);
        }
    }
    fin.close();
}

void Router::notifyMessageArrival(const std::string& toId, unsigned int numMessages)
{
    // the element to be notified is down, no need to spend system resources
    // trying to send a message
    if (isElementAvailable(toId) && dataSocketConnection(toId)) {
        NotificationData data(toId, getDataSocket(toId), MESSAGE, "", numMessages);
        if (!sendNotification(data)) {
            closeSockets(toId);
            markElementUnavailable(toId);
        }
    }
}

void Router::notifyElementAvailability(const std::string& id, bool available)
{
    logger().debug("Router::notifyElementAvailability(%s, %d)", id.c_str(), available);

    for (const auto& IdIp : ipAddress) {
        std::string toId = IdIp.first;
        //logger().debug("Router - Check need for sending notification to %s", toId.c_str());

        // Prevent from sending a availability messages to the own element.
        if (toId == id) {
            //logger().debug("Router - Discarding notification to the own unavailable element");
            continue;
        }

        // Notification messages are sent to all available elements and, when the notification refers to
        // availability of the router itself, it sends the message even to the unavailable ones so that they
        // have a chance to answer it and get removed from the unavailable_elements list.
        if ((isElementAvailable(toId) || (id == routerId) ) && controlSocketConnection(toId)) {

            // Filtering element availability messages to Proxy. Only OACs and
            // Router notifications should be sent.
            if ( toId == config().get("PROXY_ID") &&
                    (id == config().get("SPAWNER_ID") ||
                     id == config().get("LS_ID") ||
                     id == config().get("COMBO_SHELL_ID"))
               ) {
                logger().debug("Router - Discarding notification from internal network elements to Proxy");
                continue;
            }

            NotificationType type;
            if (available) {
                type = AVAILABLE;
            } else {
                type = UNAVAILABLE;
            }

            NotificationData data(toId, getControlSocket(toId), type, id, 0);
            if (!sendNotification(data)) {
                closeSockets(toId);
                markElementUnavailable(toId);
            } else {
                markElementAvailable(toId);
            }
        } else {
            logger().debug("Router - Discarding notification to element since "
                           "it is unavailable (and referred id is not router");
        }
    }
    //logger().debug("Router::notifyElementAvailability() ended");
}

void Router::closeSockets(const std::string& id)
{
    closeControlSocket(id);
    closeDataSocket(id);
}

bool Router::controlSocketConnection(const std::string& ne_id)
{
    if ( controlSockets.find(ne_id) != controlSockets.end() ) {
        // Check if socket connection is still ok. If not so, remove
        // the corresponding entry from controlSockets map and tries
        // to re-connect.
        if (!isElementAvailable(ne_id)) {
            logger().info("Router - controlSocketConnection(%s): "
                          "Element marked as unavailable. Trying to re-connect...",
                          ne_id.c_str() );
            closeSockets(ne_id);
        } else {
            logger().debug("Router - controlSocketConnection(%s): "
                           "Connection already established", ne_id.c_str() );
            return true;
        }
    } // if

    tcp::socket* sock = NULL;
    std::string ipAddr = getIPAddress(ne_id);
    int port = getPortNumber(ne_id);
    try {
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), ipAddr, boost::lexical_cast<std::string>(port));
        tcp::resolver::iterator iterator = resolver.resolve(query);
        sock = new tcp::socket(io_service);
        logger().debug("Router - controlSocketConnection(%s): "
                       "created new socket: %p.", ne_id.c_str(), sock);
        sock->connect(*iterator);
        logger().debug("Router - controlSocketConnection(%s): "
                       "connection established. ip=%s, port=%d",
                       ne_id.c_str(), ipAddr.c_str(), port);
        controlSockets[ne_id] = sock;
        return true;
    } catch(std::exception& e){
        logger().error("Router - controlSocketConnection. "
                       "Unable to connect to element %s. ip=%s, port=%d. Exception Message: %s",
                       ne_id.c_str(), ipAddr.c_str(), port, e.what());
        if (sock != NULL) delete sock;
        sock = NULL;
    }

    closeSockets(ne_id);
    markElementUnavailable(ne_id);

    return false;
}

bool Router::dataSocketConnection(const std::string& ne_id)
{
    if ( dataSockets.find(ne_id) != dataSockets.end() ) {
        // Check if socket connection is still ok. If not so, remove the
        // corresponding entry from dataSockets map and tries to re-connect.
        if (!isElementAvailable(ne_id)) {
            logger().info("Router - dataSocketConnection(%s): "
                          "Element marked as unavailable. Trying to re-connect...",
                          ne_id.c_str() );
            closeSockets(ne_id);
        } else {
            logger().debug("Router - dataSocketConnection(%s): "
                           "Connection already established", ne_id.c_str() );
            return true;
        }
    }

    tcp::socket* sock = NULL;
    std::string ipAddr = getIPAddress(ne_id);
    int port = getPortNumber(ne_id);
    try {
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), ipAddr, boost::lexical_cast<std::string>(port));
        tcp::resolver::iterator iterator = resolver.resolve(query);
        sock = new tcp::socket(io_service);
        logger().debug("Router - dataSocketConnection(%s): "
                       "created new socket: %p.", ne_id.c_str(), sock);
        sock->connect(*iterator);
        logger().debug("Router - dataSocketConnection(%s). "
                       "Connection established. ip=%s, port=%d",
                       ne_id.c_str(), ipAddr.c_str(), port);
        dataSockets[ne_id] = sock;
        return true;
    } catch(std::exception& e) {
        logger().error("Router - dataSocketConnection. "
                       "Unable to connect to element %s. ip=%s, port=%d. Exception Message: %s",
                       ne_id.c_str(), ipAddr.c_str(), port, e.what());
        if (sock != NULL) delete sock;
        sock = NULL;
    }

    closeSockets(ne_id);
    markElementUnavailable(ne_id);

    return false;
}

bool Router::sendNotification(const NotificationData& data)
{
    logger().debug("Router - Sending notification.");

    try {
        std::string cmd;
        switch (data.type) {
        case AVAILABLE:
            cmd.append("cAVAILABLE_ELEMENT ");
            cmd.append(data.element);
            cmd.append("\n");
            break;

        case UNAVAILABLE:
            cmd.append("cUNAVAILABLE_ELEMENT ");
            cmd.append(data.element);
            cmd.append("\n");
            break;

        case MESSAGE:
            cmd.append("cNOTIFY_NEW_MESSAGE ");
            cmd.append(toString(data.numMessages));
            cmd.append("\n");
            break;

        default:

            logger().error("Router - Invalid notification type '%d'.",
                         (int)data.type);
            return false;
            break;
        }

        logger().debug("Router - Sending notification (socket = %p) '%s'.",
                       data.sock, cmd.c_str());
 
        boost::system::error_code error;
        boost::asio::write(*data.sock, boost::asio::buffer(cmd),
                           boost::asio::transfer_all(), error);
        if (error) {
            logger().error("Router - sendNotification. Error transfering data");
            return false;
        }

        if (noAckMessages) {
            return true;
        }

#define BUFFER_SIZE 256
        char response[BUFFER_SIZE];
        size_t receivedBytes = data.sock->read_some( boost::asio::buffer(response), error );
        if (error && error != boost::asio::error::eof) {
            logger().error("Router - sendNotification. Invalid response. "
                    "recv returned %d ", receivedBytes );
            return false;
        }

        response[receivedBytes] = '\0'; // Assure null terminated string

        // chomp all trailing slashes from string
        int i;
        for ( i = receivedBytes - 1;
                i >= 0 && (response[i] == '\n' || response[i] == '\r'); --i ) {
            response[i] = '\0';
        }

        logger().debug("Router - sendNotification. "
                       "Received response (after chomp): '%s' bytes: %d",
                       response, receivedBytes );

        std::string answer = response;

        if (answer == NetworkElementCommon::OK_MESSAGE) {
            logger().debug("Router - Sucessfully sent notification to '%s'.",
                           data.toId.c_str());
            markElementAvailable(data.toId);
        } else {
            logger().error("Router - Failed to send notification to '%s'. (answer = %s)",
                           data.toId.c_str(), answer.c_str());
        }

    } catch (std::exception &e) {
        logger().error("Router::sendNotification - Exception: %s", e.what());
    } catch (...) {
        logger().error("Router::sendNotification - unknown exception!");
    }
    return true;
}

bool Router::isElementAvailable(const std::string &id) const
{
    pthread_mutex_lock(&unavailableIdsLock);
    bool answer = (unavailableIds.find(id) == unavailableIds.end());
    pthread_mutex_unlock(&unavailableIdsLock);
    return answer;
}

void Router::markElementUnavailable(const std::string& ne_id)
{
    pthread_mutex_lock(&unavailableIdsLock);
    Router::toNotifyUnavailability.insert(ne_id);
    unavailableIds.insert(ne_id);
    pthread_mutex_unlock(&unavailableIdsLock);
}

void Router::markElementAvailable(const std::string& ne_id)
{
    pthread_mutex_lock(&unavailableIdsLock);
    if (unavailableIds.find(ne_id) != unavailableIds.end()) {
        unavailableIds.erase(ne_id);
        toNotifyAvailability.insert(ne_id);
    }
    pthread_mutex_unlock(&unavailableIdsLock);
}

