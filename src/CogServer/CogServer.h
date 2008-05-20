/**
 * CogServer.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jan 23 16:00:19 BRT 2008
 */

#ifndef COGSERVER_H
#define COGSERVER_H

#include <vector>
#include <queue>
#include <map>
#include <pthread.h>
#include <AtomSpace.h>

#include "MindAgent.h"
#include "NetworkServer.h"
#include "CogServerRequest.h"
#include "RequestProcessor.h"

namespace opencog {

class CogServer {
    
    private:

        static AtomSpace *atomSpace;

        typedef struct {
            int frequency;
            MindAgent *agent;
        } ScheduledMindAgent;

        std::vector<ScheduledMindAgent> mindAgents;

        long cycleCount;
        bool running;

        void processMindAgents();
        void processRequests();

        pthread_mutex_t messageQueueLock;
        std::queue<CogServerRequest *> requestQueue;

        NetworkServer *networkServer;

    public:

        static AtomSpace *getAtomSpace();

        ~CogServer();
        CogServer();

        void enableNetworkServer();
        void disableNetworkServer();

        void serverLoop();
        void plugInMindAgent(MindAgent *task, int frequency);
        long getCycleCount();
        void stop();
        
        CogServerRequest *popRequest();
        void pushRequest(CogServerRequest *request);
        int getRequestQueueSize();
          
        // used for debug purposes in unit tests
        void unitTestServerLoop(int limitNumberOfCycles);
}; // class

// singleton instance (following meyer's design pattern)
CogServer& server();

}  // namespace

#endif
