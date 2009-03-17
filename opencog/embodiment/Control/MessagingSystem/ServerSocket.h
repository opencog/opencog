/**
 * ServerSocket.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 20:58:48 BRT 2007
 */

#ifndef SERVERSOCKET_H
#define SERVERSOCKET_H

#include "NetworkElement.h"

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>

namespace MessagingSystem {

class ServerSocket : public TcpSocket {

    private:

        // Used to call-back when Message arrives
        static NetworkElement *master;

        // States used in in the state machine inside onLine()
        static const int DOING_NOTHING = 0;
        static const int READING_MESSAGES = 1;
        int currentState;

        std::string currentMessage;
        std::string currentMessageFrom;
        std::string currentMessageTo;
        int currentMessageType;
        int lineCount;

    public:

        // ***********************************************/
        // Constructors/destructors

        ~ServerSocket();
        ServerSocket(ISocketHandler &handler);

        // ***********************************************/
        // General

        /**
         * Sets the class that will be notified by call-back regarding communication events.
         */
        static void setMaster(NetworkElement *ne);
          
        // ***********************************************/
        // Overriden from TcpSocket
          
        void OnLine(const std::string&);

}; // class
}  // namespace

#endif
