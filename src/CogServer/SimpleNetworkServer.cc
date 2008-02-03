#include <exceptions.h>
#include <ListenSocket.h>
#include <SocketHandler.h>

#include "SimpleNetworkServer.h"
#include "ServerSocket.h"
#include "CommandRequest.h"

using namespace opencog;

bool SimpleNetworkServer::stopListenerThreadFlag = false;

SimpleNetworkServer::~SimpleNetworkServer() {
   if (! stopListenerThreadFlag) {
       stopListenerThreadFlag = true;
       pthread_join(socketListenerThread, NULL);
   }
}

SimpleNetworkServer::SimpleNetworkServer(CogServer *cogServer, int portNumber) {
    started = false;
    stopListenerThreadFlag = false;
    this->portNumber = portNumber;
    this->cogServer = cogServer;
}

void SimpleNetworkServer::processCommandLine(CallBackInterface *callBack, const std::string &cmdLine) {

    std::string command;
    std::queue<std::string> args;

    parseCommandLine(cmdLine, command, args);
    CommandRequest *request = new CommandRequest(callBack, command, args);
    cogServer->pushRequest(request);
}

void SimpleNetworkServer::start() {

    if (started) {
        throw new RuntimeException(NULL, "Can not restart SimpleNetworkServer");
    }

    ServerSocket::setMaster(this);

    pthread_attr_init(&socketListenerAttr);
    pthread_attr_setscope(&socketListenerAttr, PTHREAD_SCOPE_PROCESS);
    pthread_attr_setinheritsched(&socketListenerAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&socketListenerAttr, PTHREAD_CREATE_DETACHED);

    pthread_create(&socketListenerThread, &socketListenerAttr, SimpleNetworkServer::portListener, &portNumber);

    started = true;

}


void *SimpleNetworkServer::portListener(void *arg) {

    int port = *((int*) arg);

	SocketHandler socketHandler;
	ListenSocket<ServerSocket> listenSocket(socketHandler);
	
    if (listenSocket.Bind(port)) {
		throw new RuntimeException(NULL, "NetworkElement - Cannot bind to port %d.", port);
    }
	
    socketHandler.Add(&listenSocket);
	socketHandler.Select(0,200);

    while (!stopListenerThreadFlag) {
        if (socketHandler.GetCount() == 0) {
    		throw new RuntimeException(NULL, "NetworkElement - Bind to port %d is broken.", port);
        }
		socketHandler.Select(0,200);
    }

    return NULL;
}

/**
 * parseCommandLine -- split string into space-separated tokens
 * @line -- input string
 * @command -- output, contains first non-whitespace part of input string
 * @args -- output, queue of space-separated tokens split from the input string.
 *
 * XXX ?? what is the purpose of this?? gnu getopt is an easier way to get
 * args from a command line.
 */
void SimpleNetworkServer::parseCommandLine(const std::string &line, std::string &command, std::queue<std::string> &args) {

    unsigned int pos1, pos2;

    pos1 = line.find(' ', 0);
    if (pos1 == line.npos) {
        command.assign(line);
        return;
    }
    command.assign(line.substr(0, pos1));

    while (pos1 != line.npos) {
        pos2 = line.find(' ', pos1 + 1);
        if (pos2 == line.npos) {
            args.push(line.substr(pos1 + 1));
        } else {
            args.push(line.substr(pos1 + 1, pos2 - pos1 - 1));
        }
        pos1 = pos2;
    }
}
