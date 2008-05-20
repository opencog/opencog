/*
 * src/CogServer/CogServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#include "Config.h"
#include "Logger.h"
#include "CogServer.h"
#include "exceptions.h"
#include "SimpleNetworkServer.h"

#include <unistd.h>
#include <time.h>
#include <sys/time.h>

using namespace opencog;
AtomSpace* CogServer::atomSpace = NULL;

CogServer::~CogServer() {
    disableNetworkServer();
}

CogServer::CogServer() : cycleCount(1), networkServer(NULL) {
    if (atomSpace != NULL) delete atomSpace;
    atomSpace = new AtomSpace();

    pthread_mutex_init(&messageQueueLock, NULL);
}

AtomSpace *CogServer::getAtomSpace() {
    return atomSpace;
}

void CogServer::enableNetworkServer() {
    if (networkServer == NULL)
        networkServer = new SimpleNetworkServer(this, config().get_int("SERVER_PORT"));
}

void CogServer::disableNetworkServer() {
    if (networkServer != NULL) {
        delete networkServer;
        networkServer = NULL;
    }
}

void CogServer::serverLoop() {
    struct timeval timer_start, timer_end;
    time_t elapsed_time;
    time_t cycle_duration = config().get_int("SERVER_CYCLE_DURATION") * 1000;

    if (networkServer != NULL) networkServer->start();
    logger().info("opencog server ready.");

    for (running = true; running;) {
        gettimeofday(&timer_start, NULL);

        if (getRequestQueueSize() != 0) processRequests();
        processMindAgents();

        cycleCount++;
        if (cycleCount < 0) cycleCount = 0;

        // sleep long enough so that the next cycle will only start
        // after config["SERVER_CYCLE_DURATION"] microseconds
        gettimeofday(&timer_end, NULL);
        elapsed_time = ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
                       (timer_end.tv_usec - timer_start.tv_usec);
        if ((cycle_duration - elapsed_time) > 0)
            usleep(cycle_duration - elapsed_time);
    }
}

void CogServer::processRequests() {
    int countDown = getRequestQueueSize();
    while (countDown != 0) {
        CogServerRequest *request = popRequest();
        request->processRequest();
        delete request;
        countDown--;
    }
}

void CogServer::processMindAgents() {
    for (unsigned int i = 0; i < mindAgents.size(); i++) {
        if ((cycleCount % mindAgents[i].frequency) == 0) {
            (mindAgents[i].agent)->run(this);
        }
    }
}

void CogServer::plugInMindAgent(MindAgent *agent, int frequency) {
    ScheduledMindAgent newScheduledAgent;
    newScheduledAgent.agent = agent;
    newScheduledAgent.frequency = frequency;
    mindAgents.push_back(newScheduledAgent);
}

long CogServer::getCycleCount() {
    return cycleCount;
}

void CogServer::stop() {
    running = false;
}

CogServerRequest *CogServer::popRequest() {

    CogServerRequest *request;

    pthread_mutex_lock(&messageQueueLock);
    if (requestQueue.empty()) {
        request = NULL;
    } else {
        request = requestQueue.front();
        requestQueue.pop();
    }
    pthread_mutex_unlock(&messageQueueLock);

    return request;
}

void CogServer::pushRequest(CogServerRequest *request) {
    pthread_mutex_lock(&messageQueueLock);
    requestQueue.push(request);
    pthread_mutex_unlock(&messageQueueLock);
}

int CogServer::getRequestQueueSize() {
    pthread_mutex_lock(&messageQueueLock);
    int size = requestQueue.size();
    pthread_mutex_unlock(&messageQueueLock);
    return size;
}

// Used for debug purposes on unit tests
void CogServer::unitTestServerLoop(int nCycles) {
    for (int i = 0; (nCycles == 0) || (i < nCycles); ++i) {
        processRequests();
        processMindAgents();
        cycleCount++;
        if (cycleCount < 0) cycleCount = 0;
    }
}

// create and return static singleton instance
CogServer& opencog::server() {
    static CogServer instance;
    return instance;
}
