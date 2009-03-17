/**
 * NetworkElement.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 16:00:19 BRT 2007
 */

#ifndef NETWORKELEMENT_H
#define NETWORKELEMENT_H

#include <set>
#include <queue>
#include <pthread.h>
#include <string>
#include <vector>
#include <SystemParameters.h>
#include <exception>

#include <Sockets/ListenSocket.h>
#include <Sockets/SocketHandler.h>

#include "Message.h"
#include "IdleTask.h"
#include "MessageCentral.h"
#include "MemoryMessageCentral.h"

#include "util/Logger.h"
#include "util/exceptions.h"

namespace MessagingSystem {

/**
 * The basic class of communications layer is a NetworkElement (NE). The idea here is that every entity 
 * (namely, every process) in the PB network should extend NetworkElement to enter the PB network and i
 * exchange information with other units.
 * 
 * If the NE is supposed to act like a server, it is strongly recommended that it delegates the control 
 * to serverLoop(), which will execute a loop as follows:
 * 
 * loop forever {
 *     retrieveMessages();
 *     while <have messages in the incomming queue> {
 *         processNextMessage();
 *     }
 *     idleTime();
 * }
 * 
 * So writing a PB server will mean writing the method to manage all kinds of request it is supposed 
 * to carry out.
 * 
 * The default implementation of idleTime() will simply put the process to sleep until a new message arrives. 
 * User's implementation may decide to do whatever it wants to before calling sleepUntilNextMessageArrives(). 
 * It is strongly recommended that user's implementation of idleTime() do not take a lot of processor time 
 * to return otherwise the server may present a large latency time to answer requests. (Note: user's 
 * implementation does not necessarily need to call sleepUntilNextMessageArrives()) 
 * 
 */
class NetworkElement {
    
    protected: 
        std::string myId; // identification of this NE in PB network (set in constructor)

         /**
         * Convenience method to encapsulate the code which acts as client when sending requests to router
	 * @return true if the command was sent successfully, and false otherwise
         */
        bool sendCommandToRouter(const std::string &cmd);
       
        /**
         * Convenience method to be called when the network element is being 
         * destroyed and should notify Router to erase ip/port information.
         */
        void logoutFromRouter();

        /**
         * This method is used to mark the an element as an unavailable one. Default 
         * implementation just add the element id to the unavailable list. If the 
         * element is the router, them all messages in router are lost, so set the
         * numberof unread messages to zero.
         *
         * Subclasses are supposed to override this method to take the corresponding 
         * actions such as stopping idle tasks.
         *
         * @param id The element that should be marked as unavailable
         */
        virtual void markAsUnavailableElement(const std::string &id);
       
        /**
         * This method is used to mark the an element as an available one. Default 
         * implementation removes the element id from the list and if it is the router
         * a hansdshake operation is executed. 
         *
         * Subclasses may override this method to 
         * take the corresponding actions when necessary.
         *
         * @param id The element that should be marked as available
         */
        virtual void markAsAvailableElement(const std::string &id);

        /**
         * Return true if the component is available or false otherwise
         *
         * @param id The id of the element to be checked
         */
        bool isElementAvailable(const std::string& id);

	/**
	 * Notify this object is already initialized (used by subclass when its initialization is done)
	 */
	void markAsInitialized();

	/**
	 * Check if subclass already marked this as initialized
	 */
	bool isInitialized();

    private:

        pthread_cond_t mainThreadBed; // main thread is put in this bed when waiting for messages retireving
        pthread_mutex_t messageQueueLock; // lock used coordinate manipulation of the messageQueue (main thread and listener thread)
        pthread_mutex_t tickLock; // lock used coordinate manipulation of the messageQueue (main thread and listener thread)
        pthread_mutex_t socketAccessLock;

	pthread_t socketListenerThread; // thread which will listen to the port
        pthread_attr_t socketListenerAttr;
        static bool stopListenerThreadFlag; // control flag used to make listener thread finish and return.
        static bool socketListenerIsReady; // control flag used to indicate listener is ready.

        int numberOfUnreadMessages; // control the number of unread messages stored in router for this network element
        int lastNumberOfUnreadMessages; // the last number of unread messages stored in router for this network element. Used for tick overflow (over process) ...
        int tickNumber; // control the number of unread messages stored in router for this network element
        int unreadMessagesCheckInterval; // controls the interval (in number of cycles) in which the serverLoop will check for messages
        int unreadMessagesRetrievalLimit; // sets the limit of messages per request to be retrieved from the router (-1 for unlimitted)

        std::string ipAddress; // ip of the network card this server will listen to
        int portNumber; // port this NE will listen to (set in constructor)

        MemoryMessageCentral messageCentral;
		
        std::string routerID;
        std::string routerIP;
        int routerPort;

        int idleCyclesPerTick;
		
        typedef struct {
            int frequency;
            IdleTask *task;
        } ScheduledIdleTask;
        std::vector<ScheduledIdleTask> idleTasks; // keeps idle tasks and respective "frequency" parameters (tasks are acted once per "frequency" idle cycles)
        int idleCycles;

        char threadArgs[256]; // used to pass arguments to static method called by listener thread
        int sock;

        /**
         * Used to hold the ids of the NetworkElements that are currently
         * unavailable. To be removed from the list an IS_AVAILABLE_COMPONENT <id> 
         * message is sent to the router, and with an 'OK'answer the component
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
         * Convenience method used to simplify the flow of retrieveMessages(). It creates the proper message to be sent to router and send it away.
         */
        void requestUnreadMessages(int limit);

        /**
         * Convenience method called in constructor to perform initial handshaking with router.
         */
        void handshakeWithRouter();


    public:

        static Control::SystemParameters parameters;

	static const std::string OK_MESSAGE;
	static const std::string FAILED_MESSAGE;
	static const std::string FAILED_TO_CONNECT_MESSAGE;

	bool noAckMessages; // flag to define if the protocol should use ACK messages (OK,FAILED) or not.

        // ***********************************************/
        // Constructors/destructors

        virtual ~NetworkElement();

        /**
         * Builds thje object and perform the handshake with the Router:
         *    (a) Request access passing an identification string and a port number (the port this NE will listen to)
         *    (b) Receive an OK or a failure message indicating that this NE could not join PB network.
         *    (c) Starts a secondary thread which will listen to the given port to process commands from the Router 
         *        (the main thread returns from constructor)
         *
         * @param myId A string which will identify this NE. Other NEs may use to string to discover this NE's actual id.
         * @param portNumber The port number this NE will listen to
         */
        NetworkElement(const Control::SystemParameters &params, const std::string &myId, const std::string &ip, int portNumber);

        /**
         * Empty constructor which does not initialize (ie, does not contact router for handshaking).
         * Useful to build unit tests.
         */
        NetworkElement();

        /**
         * Initializes the object and perform the handshake with the Router:
         *    (a) Request access passing an identification string and a port number (the port this NE will listen to)
         *    (b) Receive an OK or a failure message indicating that this NE could not join PB network.
         *    (c) Starts a secondary thread which will listen to the given port to process commands from the Router 
         *        (the main thread returns from constructor)
         *
         * @param myId A string which will identify this NE. Other NEs may use to string to discover this NE's actual id.
         * @param portNumber The port number this NE will listen to
         */
        void initialize(const Control::SystemParameters &params, const std::string &myId, const std::string &ip, int portNumber);

        // ***********************************************/
        // API

        /**
         * Return true if there are new unread messages waiting in the router. It is a local check, no communication is actually performed. This method just returns a local boolean state variable which is set assynchronously by the router when a new message to this NE arrives.
         *
         * @return true if there are new unread messages waiting in the router or false otherwise.
         */
        bool haveUnreadMessage();

        /**
         * Contacts the Router and retrieve at most N unread messages.
         *
         * @param limit Max number of messages which will be retrieved
         */
        bool retrieveMessages(int limit = -1);

        /**
         * Send a Message. Information about target is encapsulated inside the Message.
         *
         * @param msg The message to be sent.
         */
        virtual bool sendMessage(Message &msg);

        /**
         * Puts the main thread to sleep (no busy waiting) until Router notifies the arrival of a new Message.
         */
        void sleepUntilNextMessageArrives();

        /**
         * Convenience method to make NE act like an usual server. 
         *
         * loop forever {
         *     serverCycle()
         * }
         */
        void serverLoop();

        /**
         * Convenience method to make NE act like an usual server. 
         *
         *     while haveUnreadMessage() {
         *         processNextMessage();
         *     } otherwise {
         *         idleTime();
         *     }
         */
        bool serverCycle(bool waitTick = false);


        /**
         * Convenience method to make NE act like an usual server in OPC. 
         *
         * loop forever {
         *     serverCycle()
         * 	   wait_until_new_tick_arrive()
         * }
         */
        void tickedServerLoop();

		
        /**
         * Called when the NE is not precossing Messages. Subclasses are supposed to override this to provide
         * a sensible behavior (default implementation just put NE to sleep).
         */
        virtual void idleTime();

        /**
         * Called when NE retrieve a Message from router to perform some useful proceesing on it. Subclasses
         * are supposed to override this to perform something useful (default implementation just outputs
         * Message contents to stdout).
         * @return true, if the NetworkElement must exit (e.g. it received a SAVE_AND_EXIT message)
         */
        virtual bool processNextMessage(Message *message);

        /**
         * This method is called inside serverLoop() just before ir enters in the loop itself. 
         * Default implementation just initializes the main logger.
         */
        virtual void setUp();

        /**
         * Used to plug in idleTasks which will be called by idle time according to the frequency parameter.
         * When in serverLoop(), NetworkElement enters in idleTime() every time it doesn't have messages to
         * process. Every call to idleTime() is considered "a cycle". frequency is given in number of such
         * cycles. The passed task will be performed once every <frequency> cycles.
         *
         * Tasks are executed in the same order they have been plugged in.
         *
         * @param task The task which will be acted
         * @param frequency Task will be acted once every <frequency> idle cycles.
         */
        void plugInIdleTask(IdleTask *task, int frequency);

        // END OF API

        /**
         * Convenience method to parse command lines into commands and arguments.
         */
        static void parseCommandLine(const std::string &line, std::string &command, std::queue<std::string> &args);

        const std::string &getIPAddress();
        int getPortNumber();
        const std::string &getID();


        // ***********************************************/
        // ServerSocket call-back

        // Although the following methods are public, they are not supposed to be called
        // by other classes other than ServerProxy. They are called on assynchronous call-back
        // from requests arriving in the listened port.

        /**
         * This method is supposed to be called by ServerSocket to notify NE that a new message has arrived in the Router targeted to this NE (the message is still in the Router). It shaw not be called by anyone else.
         */
        void newMessageInRouter(int numMessages);

        /**
         * This method is supposed to be called by ServerSocket to notify NE that at
         * least one new message has been actually delivered by the Router. It shaw 
         * not be called by anyone else.
         */
        void newMessageRead(const std::string &from, const std::string &to, int type, const std::string &msg);

        /**
         * This method is supposed to be called by ServerSocket to notify NE that all
         * unread messages have been received from the Router. It shaw not be called 
         * by anyone else.
         */
        void noMoreMessages();

        /**
         * This method is supposed to be called by ServerSocket to notify NE that the given
         * element is unavailable to receive messages. It shaw not be called by anyone else.
         */
        void unavailableElement(const std::string &id);

        /**
         * This method is supposed to be called by ServerSocket to notify NE that the given
         * element is available to receive messages (after being unavailable for a while since
         * by default all components are assume to be available at start up).
         * It shaw not be called by anyone else.
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

}; // class
}  // namespace

#endif
