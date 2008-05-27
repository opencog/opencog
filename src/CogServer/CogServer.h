/*
 * src/CogServer/CogServer.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
 *            Gustavo Gama <gama@vettalabs.com>
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
    std::vector<MindAgent *> inputHandlers;

    long cycleCount;
    bool running;

    void processInput();
    void processMindAgents();
    void processRequests();

    pthread_mutex_t messageQueueLock;
    std::queue<CogServerRequest *> requestQueue;

    NetworkServer *networkServer;

public:

    static AtomSpace *getAtomSpace();

    ~CogServer();
    CogServer(void);

    void enableNetworkServer(void);
    void disableNetworkServer(void);

    void serverLoop(void);
    void plugInMindAgent(MindAgent *task, int frequency);
    void plugInInputHandler(MindAgent *task);
    long getCycleCount(void);
    void stop(void);
    
    CogServerRequest *popRequest(void);
    void pushRequest(CogServerRequest *request);
    int getRequestQueueSize(void);
      
    // used for debug purposes in unit tests
    void unitTestServerLoop(int limitNumberOfCycles);
}; // class

// singleton instance (following meyer's design pattern)
CogServer& server();

}  // namespace

#endif
