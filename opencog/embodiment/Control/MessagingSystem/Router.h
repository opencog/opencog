/*
 * opencog/embodiment/Control/MessagingSystem/Router.h
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


#ifndef ROUTER_H
#define ROUTER_H

#include <map>
#include <set>
#include <queue>
#include <string>
#include <vector>
#include <time.h>
#include <exception>
#include <opencog/embodiment/Control/EmbodimentConfig.h>
#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include "MessageCentral.h"
#include "MemoryMessageCentral.h"
#include "Message.h"
#include <pthread.h>


#include <boost/asio.hpp>
using boost::asio::ip::tcp;

namespace opencog { namespace messaging {

class RouterServerSocket;

// Notification types sent by the router
enum NotificationType {
    MESSAGE,
    AVAILABLE,
    UNAVAILABLE
};

/**
 * Struct used to pass information to the thread used to send notifications.
 */
struct NotificationData {

    std::string toId;
    tcp::socket *sock;
    NotificationType type;
    std::string element;
    unsigned int numMessages;

    NotificationData(const std::string& _toId,
                     tcp::socket* _sock,
                     NotificationType _type,
                     const std::string& _element,
                     unsigned int _numMessages)
        : toId(_toId), sock(_sock), type(_type),
          element(_element), numMessages(_numMessages) {}

}; // struct NotificationData

typedef std::map<std::string, tcp::socket*> Id2SocketMap;

static const std::string dummyIP = std::string();

/**
 * Router is not supposed to be used directly by OpenCog (OC)
 * entities. All communication with it should be performed by
 * NetworkElement's (NE) internals. NE encapsulates the protocol used
 * to communicate with Router but in the case you want/need to
 * simulate a NE communicating directly with the router, you may find
 * useful the protocol description provided below.
 *
 * There are five relevant use cases for the protocol:
 *
 * (1) YourEntity starts a handshake with router to enter OC network
 * (no authentication is required at this point. It is just to allow
 * other entities to know about YourEntity)
 *
 * (2) YourEntity wants to send a Message to some other entity (it
 * will always contact the router to do so, regardless who is the
 * actual target for that message).
 *
 * (3) YourEntity is asynchronously notified by router that there are
 * unread messages to it waiting in router's queue. YourEntity will
 * need to be listening to a given port to receive such a message.
 *
 * (4) YourEntity decides it wants to retrieve unread messages from
 * router so it sends a proper request (this request will be answered
 * asynchronously by router).
 *
 * (5) YourEntity is asynchronously contacted by the router to receive
 * requested unread messages. YourEntity will need to be listening to
 * a given port to receive the messages.
 *
 * (1)
 *
 * Router listens to the port 5005.
 * Open a connection to this port and send the following command:
 *
 * LOGIN <YOUR_ENTITY> <ip> <port>
 *
 * All communication is performed in line-based protocol. So everything
 * should go with an explicit '\\n'. (this is true from every command or
 * data line). YOUR_ENTITY is a literal string used to identify YourEntity in
 * OC network. <ip> and <port> will be used to connect to your entity.
 * Please use ports > 5100 (obviously 5005 is already taken).
 *
 * router will answer in the same connection with
 *
 * OK
 *
 * or an error message. Then you can just close the connection.
 *
 * (2)
 *
 * Open a connection to router
 * send the command:
 *
 * NEW_MESSAGE <from> <to> <type> <no. lines>
 *
 * <from> and <to> are the string id of source/target. <from> will be YOUR_ENTITY
 * <to> may be either SPAWNER or any othe OC element you want to contact
 * <type> is message type, e.g. type = 1 is stringMessage.
 * <no. lines> is the number of lines in your message. All messages are broken into
 * lines and sent line by line (lines should not be larger than 1024 chars).
 *
 * So send each line of message.
 *
 * So router will answer with
 *
 * OK
 *
 * or an error message. The message is sent. You can close connection.
 *
 * (3)
 *
 * YourEntity is supposed to be listening to the port passed in handshake. Router will
 * connect to this port and send:
 *
 * cNOTIFY_NEW_MESSAGE
 *
 * the prefix 'c' is not a typo :-)
 * routers closes the connection
 *
 * (4) YourEntity decides it want to retrieve unread messages. It will
 * connect to router's port and send:
 *
 * REQUEST_UNREAD_MESSAGES YOUR_ENTITY <limit>
 *
 * <limit> is the maximum number of messages you expect to be retrieved. -1
 * makes it to retrieve all unread messages. So router may answer
 *
 * OK
 *
 * or an error message. YourEntity closes the connection.
 *
 * (5) router connects to YourEntity's port and send:
 *
 * cSTART_MESSAGE <from> <to> <type>
 *
 * then routers sends one or more lines of data (the message itself). note
 * that each data line is prefixed with 'd'.
 * Then the router sends either:
 *
 * cSTART_MESSAGE <from> <to> <type>
 *
 * To start sending another message. Or:
 *
 * cNO_MORE_MESSAGES
 *
 * to denote that all messages have been sent already.
 *
 * An important warning is that YourEntity should not request message retrieving if it
 * was not notified first about new messages to it in router. The C++ interface provide
 * this automatically but you must take care of it as you are bypassing
 * the interface.
 *
 * (6) Router connects to YourEntity's port and send:
 *
 * cAVAILABLE_ELEMENT <elementId>
 * or
 * cUNAVAILABLE_ELEMENT <elementId>
 *
 * to indicate a specific Network Element became available or unavailable
 */
class Router
{

private:

    // NOTE: Uncomment this if need to insert an exception into router
    //bool exception;

    std::map<std::string, std::string> ipAddress;
    std::map<std::string, int> portNumber;
    boost::asio::io_service io_service;
    Id2SocketMap controlSockets;
    Id2SocketMap dataSockets;

    static std::vector<RouterServerSocket*> serverSockets;

    /**
     * Control the run() function main loop. Once set false, the Router will
     * end its execution.
     */
    bool running;

    /**
     * Counter used to cycle router availability notification
     */
    time_t lastNotifyTimestamp;

    /**
     * Store the component ids that are temporary unavailable for some
     * reason.
     */
    std::set<std::string> unavailableIds;

    /**
     * Store the ids of the elements just marked as available or unavailable
     * and a notification should be sent to everyone
     */
    std::set<std::string> toNotifyAvailability;
    std::set<std::string> toNotifyUnavailability;

    // Interface to messageQueues
    MessageCentral *messageCentral;

    std::string routerId;
    int routerPort;
    int routerAvailableNotificationInterval;

    /**
     * Load persisted information about the elements previously connected to
     * the router (id, ip and port). With such information the router will
     * notify these elements that the component has recoverd from an
     * exceptions crash.
     *
     * @param fileName A std::string with the name (full path) of the file
     *                 with the persisted information.
     */
    void recoveryFromPersistedData(const std::string& filename);

    /**
     * Notify an element about some situation:
     * a new message, an available or unavailable element.
     *
     * This method has to be called for all know elements, since there
     * is no broadcast available for TCP protocol.
     *
     * @param data notification data, a struct with the notification message + receiver Id and socket
     * @return true if socket connection is still ok after trying to send the notification
     */
    bool sendNotification(const NotificationData& data);

    /**
     * Coordinates access to unavailableIds
     */
    mutable pthread_mutex_t unavailableIdsLock;

    pthread_t socketListenerThread; // thread which will listen to the router port
    pthread_attr_t socketListenerAttr; // thread attributes
    static bool stopListenerThreadFlag; // control flag used to make listener thread finish and return.

    /**
     * This method takes control of the thread started to listen to the router port
     *
     * @param arg A pointer to an integer with the port number the thread is supposed to listen to.
     */
    static void *portListener(void *arg);

    /**
     * Convenience method called in constructor to delegate router server socket listening to another thread
     */
    void startListener();

    /**
     * Notify all components that a given component is available to receive
     * messages again.
     *
     * IMPORTANT: Filters are applied to notifications sent to Proxy. Only
     * the ones related to OAC are considered.
     *
     * @param id The id of element that is available again.
     * @param available True if the element is available and false
     *        otherwise. Default = true.
     */
    void notifyElementAvailability(const std::string& id, bool available = true);

public:

#ifdef NO_ERROR
#undef NO_ERROR
#endif
    static const int NO_ERROR = 0;
    static const int ID_EXISTS = 1;
    static const int PORT_EXISTS = 2;
    static const int HAS_PENDING_MSGS = 3;

    //! Flag to define if the protocol should use ACK messages (OK,FAILED) or not.
    bool noAckMessages;

    // ***********************************************/
    // Constructors/destructors
    
    Router();
    ~Router();

    // ***********************************************/
    // public interface

    /** Check if the id is known.
     * @warning side effect: creates a message queue for the ID if it is unknown.
     * @return whether the id of element is known
     */
    bool knownID(const std::string &id);

    //! Get the listening port number of a NetworkElement.
    int getPortNumber(const std::string &id) const;

    //! Get the IP address of NetworkElement.
    const std::string &getIPAddress(const std::string &id) const;

    //! Retrieve network socket for sending control messages to network element?
    tcp::socket* getControlSocket(const std::string &id) const;

    //! Retrieve network socket for sending data messages to network element?
    tcp::socket* getDataSocket(const std::string &id) const;

    //! Close network socket for sending control messages to network element?
    void closeControlSocket(const std::string &id);

    //! Close network socket for sending data messages to network element?
    void closeDataSocket(const std::string &id);

    // Close control and data sockets of network element id
    void closeSockets(const std::string& id);

    /** Remove network element from the embodiment system and clean up sockets
     * and queues associated with it.
     */
    void removeNetworkElement(const std::string &strId);

    //! Clear all pending messages for NetworkElement strId
    void clearNetworkElementMessageQueue(const std::string &strId);

    //! Add new network element to the embodiment system
    int addNetworkElement(const std::string &strId, const std::string &strIp, int port);

    MessageCentral* getMessageCentral();

    /**
     * Main loop. Infinite loop listening to a port.
     */
    void run();

    /**
     * Exit the router. Should be called only by the router server socket
     */
    void shutdown();

    /**
     * Save id, ip and port information of all available elements corrently
     * logged in the Router. This information will be used in router
     * recovery strategies
     */
    void persistState() const;

    /**
     * Notify a component that a message has been sent to it.
     *
     * @param toId the id of the receiver of the notification.
     * @numMessages the number of messages received (or the number of
     * messages in router when handshaking)
     */
    void notifyMessageArrival(const std::string& toId, unsigned int numMessages);

    /**
     * Return true if the component is available or false otherwise.
     * NetworkElements are unavailable when notification messages fail or
     * when restarting the router using a saved set of connections.
     *
     * @param id The id of the element to be checked
     */
    bool isElementAvailable(const std::string &id) const;

    /** Mark the network element with given id as unavailable.
     * This will cause Router to send UNAVAILABLE_ELEMENT to all other network
     * elements.
     **/
    void markElementUnavailable(const std::string& ne_id);

    /** Mark the network element with given id as available.
     * This will cause Router to send AVAILABLE_ELEMENT to all other network
     * elements.
     **/
    void markElementAvailable(const std::string& ne_id);

    /**
     * Set stopListenerThread tag to true and stop listening for messages
     */
    void stopListenerThread();

    /**
    * Establish control connection to the network element with the given id, if
    * not yet connected.
    * Also updates internal map from network element ids to control sockets.
    * @return true if the connection is established.
    */
    bool controlSocketConnection(const std::string& ne_id);

    /**
    * Estabsish data connection to the network element with the given id, if not
    * yet connected.
    * Also updates internal map from network element ids to data sockets.
    * @return true if the connection is established.
    */
    bool dataSocketConnection(const std::string& ne_id);

    /**
     * Check if NetworkElment is still listenning to its tcp port.
     */
    static bool isListenerThreadStopped() {
        return !stopListenerThreadFlag;
    }

}; // class Router
} } // namespace opencog::messaging

#endif
