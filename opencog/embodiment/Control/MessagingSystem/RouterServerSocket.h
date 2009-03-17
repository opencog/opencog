/**
 * RouterServerSocket.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Fri Jun 22 15:20:43 BRT 2007
 */

#ifndef ROUTERSERVERSOCKET_H
#define ROUTERSERVERSOCKET_H

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

#include "Router.h"

namespace MessagingSystem {

class RouterServerSocket : public TcpSocket {

    private:

        int currentState;
        static const int WAITING_COMMAND = 1;
        static const int READING_MESSAGE = 2;

        std::string currentMessageText;
        std::string currentMessageFrom;
        std::string currentMessageTo;
        int currentMessageType;
        int currentMessageSize;
        int messageLinesCountdown;
        bool firstLineOfMessageFlag;

        // Used to call-back when messages arrive
        static Router *master;

        std::map<std::string, int> sockets;

        void parseCommandLine(const std::string &line, std::string &command, std::queue<std::string> &args);
        void sendAnswer(const std::string &msg);
        void addNetworkElement(const std::string &id, const std::string &ip, int port);
        void sendRequestedMessages(const std::string &id, int limit, bool useHttpRequest = false);
        bool sendHttpRequest(const std::string& messageFrom, const std::string& messageTo, const std::string& messageText);
        void storeNewMessage();

    public:

        // ***********************************************/
        // Constructors/destructors

        ~RouterServerSocket();
        RouterServerSocket(ISocketHandler &handler);

        // ***********************************************/
        // General

        /**
         * Sets the class that will be notified by call-back regarding communication events.
         */
        static void setMaster(Router *router);
          
        // ***********************************************/
        // Overriden from TcpSocket
          
        void OnLine(const std::string &line);

}; // class
}  // namespace

#endif
