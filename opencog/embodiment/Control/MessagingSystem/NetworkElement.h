/*
 * opencog/embodiment/Control/MessagingSystem/NetworkElement.h
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


#ifndef NETWORKELEMENT_H
#define NETWORKELEMENT_H

#include <set>
#include <queue>
#include <pthread.h>
#include <string>
#include <vector>
#include <exception>

#include "Message.h"
#include "MessageCentral.h"
#include "MemoryMessageCentral.h"

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

#include <boost/asio.hpp>
using boost::asio::ip::tcp;

namespace opencog { namespace messaging {

    class ServerSocket;

/**
 * The basic class of communications layer is a NetworkElement (NE). The idea
 * here is that every entity (namely, every process) in the embodiment network
 * should have a NetworkElement member to participate and exchange information
 * with other units.
 *
 * So a embodiment server should have a loop to check and process all incoming
 * messages of the type it is supposed to receive.
 */
class NetworkElement
{

protected:
    //! Identification of this NE in embodiment network (set in constructor)
    std::string myId;

    /**
     * This method is used to mark an element unavailable. Default
     * implementation just add the element id to the unavailable list. If the
     * element is the router, then all messages in router are lost, so set the
     * numberof unread messages to zero.
     *
     * Subclasses are supposed to override this method to take the corresponding
     * actions such as stopping idle tasks.
     *
     * @param id The element that should be marked as unavailable
     */
    void markAsUnavailableElement(const std::string &id);

    /**
     * This method is used to mark an element as available. Default
     * implementation removes the element id from the unavailable list and if
     * it is the router a handshake operation is executed.
     *
     * Subclasses may override this method to take the corresponding actions
     * when necessary.
     *
     * @param id The element that should be marked as available
     */
    void markAsAvailableElement(const std::string &id);

    /**
     * Notify this object is already initialized (used by subclass when its initialization is done)
     */
    void markAsInitialized();

    /**
     * Check if subclass already marked this as initialized
     */
    bool isInitialized();

private:

    //! main thread is put in this bed when waiting for messages retrieving
    pthread_cond_t mainThreadBed;
    //! lock used to coordinate manipulation of the messageQueue (main thread and listener thread)
    pthread_mutex_t messageQueueLock;
    //! lock used to coordinate manipulation of the messageQueue (main thread and listener thread)
    pthread_mutex_t tickLock;
    pthread_mutex_t socketAccessLock;

    pthread_t socketListenerThread; // thread which will listen to the port
    pthread_attr_t socketListenerAttr;
    //! control flag used to make listener thread finish and return.
    static bool stopListenerThreadFlag;
    //! control flag used to indicate listener is ready.
    static bool socketListenerIsReady;

    //! control the number of unread messages stored in router for this network element
    int numberOfUnreadMessages;
    //! The last number of unread messages stored in router for this network
    //! element. Used for tick overflow (over process) ...
    int lastNumberOfUnreadMessages; 
    //! control the number of unread messages stored in router for this network element
    int tickNumber;

    std::string ipAddress; // ip of the network interface this server will listen to
    int portNumber; // port this NE will listen to (set in constructor)

    MemoryMessageCentral messageCentral;

    std::string routerID;
    std::string routerIP;
    int routerPort;

    char threadArgs[256]; // used to pass arguments to static method called by listener thread
    boost::asio::io_service io_service;
    tcp::socket* sock;

    static std::vector<ServerSocket*> serverSockets;

    /**
     * Used to hold the ids of the NetworkElements that are currently
     * unavailable. To be removed from the list, an IS_AVAILABLE_COMPONENT <id>
     * message is sent to the router, and with an 'OK' answer the component
     * id is removed.
     *
     * IMPORTANT:
     * This list can also hold the router's id if it goes down. In that case
     * the remove action remains the same since if the router is able to
     * receive the message it is up and will answer 'OK'
     */
    std::set<std::string> unavailableElements;

    /**
     * Flag that indicates the subclass of NetworkElement is already initialized
     */
    bool subclass_initialized;

    void addTick();
    int getTick();

    /**
     * This method takes control of the thread started to listen to the port this
     * NetworkElement is supposed to listen to.
     *
     * @param arg A pointer to an integer with the port number the thread is supposed
     * to listen to.
     */
    static void *portListener(void *arg);

    /**
     * Convenience method called in constructor to delegate socket listening to another thread
     * @return true if listener was initialized and it's ready for usage. False otherwise
     */
    bool startListener();

    /**
     * Convenience method used to simplify the flow of retrieveMessages(). It
     * creates the proper message to be sent to router and sends it away.
     */
    void requestUnreadMessages(int limit);

    /**
     * Convenience method called in constructor to perform initial handshaking with router.
     */
    void handshakeWithRouter();


public:
    //! flag to define if the protocol should use ACK messages (OK,FAILED) or not.
    bool noAckMessages;

    // ***********************************************/
    // Constructors/destructors
    virtual ~NetworkElement();

    /**
     * Builds thje object and perform the handshake with the Router:
     *  #- Request access passing an identification string and a port number
     * (the port this NE will listen to) 
     *  #- Receive an OK or a failure message indicating that this NE could
     * not join the embodiment network
     *  #- Starts a secondary thread which will listen to the given port to
     * process commands from the Router (the main thread returns from
     * constructor)
     *
     * @param myId A string which will identify this NE. Other NEs may use to
     * string to discover this NE's actual id.
     * @param portNumber The port number this NE will listen to
     */
    NetworkElement(const std::string &myId, const std::string &ip, int portNumber);

    /**
     * Empty constructor which does not initialize (ie, does not contact router for handshaking).
     * Useful to build unit tests.
     */
    NetworkElement();

    /**
     * Initializes the object and perform the handshake with the Router:
     *  #- Request access passing an identification string and a port number
     *  (the port this NE will listen to)
     *  #- Receive an OK or a failure message indicating that this NE could not
     *  join the embodiment network
     *  #- Starts a secondary thread which will listen to the given port to
     *  process commands from the Router (the main thread returns from constructor)
     *
     * @param myId A string which will identify this NE. Other NEs may use to
     * string to discover this NE's actual id.
     * @param portNumber The port number this NE will listen to
     */
    void initialize(const std::string &myId, const std::string &ip, int portNumber);

    /**
     * Convenience method to be called when the network element is being
     * destroyed and should notify Router to erase ip/port information.
     */
    void logoutFromRouter();

    // ***********************************************/
    // API

    /**
     * Return true if the component is available or false otherwise
     *
     * @param id The id of the element to be checked
     */
    bool isElementAvailable(const std::string& id);

    /**
     * Return true if there are new unread messages waiting on the router.
     * It is a local check, no communication is actually performed.
     * This method just returns a local boolean state variable which is set
     * assynchronously by the router when a new message to this NE arrives.
     *
     * @return true if there are new unread messages waiting on the router or
     * false otherwise.
     */
    bool haveUnreadMessage();

    /**
     * Contacts the Router and retrieve at most N unread messages.
     *
     * @param limit Max number of messages which will be retrieved. If
     *              negative all messages are retrieved.
     */
    bool retrieveMessages(int limit = -1);

    /**
     * Send a Message. Information about target is encapsulated inside the Message.
     *
     * @param msg The message to be sent.
     */
    virtual bool sendMessage(Message &msg);

    /**
     * Convenience method to encapsulate the code which acts as client when
     * sending requests to router.
     * @return true if the command was sent successfully, and false otherwise
     */
    virtual bool sendCommandToRouter(const std::string &cmd);

    // END OF API

    const std::string &getIPAddress();
    int getPortNumber();
    const std::string &getID();


    // ***********************************************/
    // ServerSocket call-back

    // Although the following methods are public, they are not supposed to be called
    // by other classes other than ServerProxy. They are called on assynchronous call-back
    // from requests arriving in the listened port.

    /**
     * This method is supposed to be called by ServerSocket to notify NE that
     * a new message has arrived in the Router targeted to this NE (the message
     * is still on the Router). It should not be called by anyone else.
     */
    void newMessageInRouter(int numMessages);

    /**
     * This method is supposed to be called by ServerSocket to notify NE that at
     * least one new message has been actually delivered by the Router. It
     * should not be called by anyone else.
     */
    void newMessageRead(const std::string &from, const std::string &to, int type, const std::string &msg);

    /**
     * This method is supposed to be called by ServerSocket to notify NE that all
     * unread messages have been received from the Router. It should not be called
     * by anyone else.
     */
    void noMoreMessages();

    /**
     * This method is supposed to be called by ServerSocket to notify NE that the given
     * element is unavailable to receive messages. It should not be called by anyone else.
     */
    void unavailableElement(const std::string &id);

    /**
     * This method is supposed to be called by ServerSocket to notify NE that the given
     * element is available to receive messages (after being unavailable for a while since
     * by default all components are assume to be available at start up).
     * It should not be called by anyone else.
     */
    void availableElement(const std::string &id);

    /**
     * Set stopListenerThread tag to true and stop listerning the messages
     */
    void stopListenerThread();

    /**
    * Open a connection to Router
    * Return true if a connection was already stabilished or false if errors occurred
    */
    bool connectToRouter( void );

    /**
    * Sends a string message to router using a persistent socket connection
    */
    std::string sendMessageToRouter( const std::string& message );

    /**
     * Methods for accessing incoming queue
     */
    unsigned int getIncomingQueueSize();
    bool isIncomingQueueEmpty();
    Message* popIncomingQueue();

    /**
     * Method to check if NetworkElment is still listenning to its tcp port
     */
    static bool isListenerThreadStopped() { 
        return !stopListenerThreadFlag;
    }

}; // class
} } // namespace opencog::messaging

#endif
