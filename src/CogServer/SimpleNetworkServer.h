/**
 * SimpleNetworkServer.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Tue Jan  23 16:17:00 BRT 2008
 */

#ifndef SIMPLENETWORKSERVER_H
#define SIMPLENETWORKSERVER_H

#include "CogServer.h"
#include "NetworkServer.h"
#include "CallBackInterface.h"
#include <pthread.h>

#include <string>
#include <queue>

namespace opencog {

class SimpleNetworkServer : public NetworkServer
{
    private:

        bool started;
        static bool stopListenerThreadFlag;
        int portNumber;
        CogServer *cogServer;

        pthread_t socketListenerThread; // thread which will listen to the port
        pthread_attr_t socketListenerAttr;

        static void *portListener(void *arg);
        static void parseCommandLine(const std::string &cmdLine,
                                     std::string &command,
                                     std::queue<std::string> &args);

        // Buffer to store raw data streaming in over a socket. The data
        // needs to be buffered because the XML parser wasn't designed to 
        // run in streaming mode, it can only batch the requests :-(
        //
        // XXX This is fundamentally wrong; the data buffer should be per-thread,
        // and there should be one thread per client. (i.e. there should be at most
        // one buffer per client) However, that is not currently implemented, 
        // so this needs to be fixed.
        std::string buffer;
    public:

        ~SimpleNetworkServer();
        SimpleNetworkServer(CogServer *cogServer, int portNumber);

        void processCommandLine(CallBackInterface *callBack, const std::string &line);
        void processData(CallBackInterface *callBack, const char *, size_t);

        void start();

}; // class
}  // namespace

#endif
