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
#include "CogServerSetup.h"
#include "NetworkServer.h"
#include "CogServerRequest.h"
#include "RequestProcessor.h"

namespace opencog {

class CogServerSetup;

class CogServer {
    
    private:

        static AtomSpace *atomSpace;

        typedef struct {
            int frequency;
            MindAgent *agent;
        } ScheduledMindAgent;

        std::vector<ScheduledMindAgent> mindAgents;

        long cycleCount;

        void processMindAgents();
        void processRequests();

        pthread_mutex_t messageQueueLock;
        std::queue<CogServerRequest *> requestQueue;

        CogServerSetup *initializer;
        NetworkServer *networkServer;

    public:

        ~CogServer();
        CogServer();

        static AtomSpace *getAtomSpace();
        void serverLoop();
        void plugInMindAgent(MindAgent *task, int frequency);
        long getCycleCount();
        
        CogServerRequest *popRequest();
        void pushRequest(CogServerRequest *request);
        bool getRequestQueueSize();
        void setNetworkServer(NetworkServer *networkServer);
          
        // used for debug purposes in unit tests
        void unitTestServerLoop(int limitNumberOfCycles);
}; // class
}  // namespace

#endif
