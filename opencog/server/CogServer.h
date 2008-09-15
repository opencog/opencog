/*
 * opencog/server/CogServer.h
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

#ifndef _OPENCOG_COGSERVER_H
#define _OPENCOG_COGSERVER_H

#include <vector>
#include <queue>
#include <map>

#include <pthread.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/BaseServer.h>
#include <opencog/server/CogServerRequest.h>
#include <opencog/server/MindAgent.h>
#include <opencog/server/NetworkServer.h>
#include <opencog/server/RequestProcessor.h>

namespace opencog
{

class CogServer : public BaseServer
{

private:

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

    static CogServer* createInstance(void);

    CogServer(void);
    virtual ~CogServer(void);

    virtual void enableNetworkServer(void);
    virtual void disableNetworkServer(void);

    virtual void serverLoop(void);
    virtual void plugInMindAgent(MindAgent *task, int frequency);
    virtual void plugInInputHandler(MindAgent *task);
    virtual long getCycleCount(void);
    virtual void stop(void);

    virtual CogServerRequest *popRequest(void);
    virtual void pushRequest(CogServerRequest *request);
    virtual int getRequestQueueSize(void);

    // used for debug purposes in unit tests
    virtual void unitTestServerLoop(int limitNumberOfCycles);
}; // class

}  // namespace

#endif // _OPENCOG_COGSERVER_H
