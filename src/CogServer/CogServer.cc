/**
 * CogServer.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 16:00:19 BRT 2007
 */

#include "CogServer.h"

#include <exceptions.h>
#include <unistd.h>

using namespace opencog;

AtomSpace *CogServer::atomSpace = NULL;

CogServer::~CogServer() {
    delete initializer;
    delete networkServer;
}

CogServer::CogServer() {
    cycleCount = 1;
    pthread_mutex_init(&messageQueueLock, NULL);
    initializer = new CogServerSetup();
    networkServer = NULL;
    if (atomSpace != NULL) {
        delete atomSpace;
    }
    atomSpace = new AtomSpace();
}

AtomSpace *CogServer::getAtomSpace() {
    return atomSpace;
}

/**
 * Used for debug purposes in unit tests
 */
void CogServer::unitTestServerLoop(int limitNumberOfCycles) {

    do {
        if ((limitNumberOfCycles >= 0) && (cycleCount > limitNumberOfCycles)) {
            return;
        }

        processRequests();
        processMindAgents();

        cycleCount++;
        if (cycleCount < 0) {
            cycleCount = 0;
        }

    } while(true);
}

void CogServer::setNetworkServer(NetworkServer *networkServer) {
    if (this->networkServer != NULL) {
        throw new RuntimeException(NULL, "Can not reset NetworkServer");
    }
    this->networkServer = networkServer;
}

static char * heap_bottom;

void CogServer::serverLoop()
{
    initializer->setUp(this);
    if (networkServer != NULL) {
        networkServer->start();
    }
heap_bottom=(char *)sbrk(0);

    do {
        // Avoid hard spinloop when there's no work.
        int reqQueSize = getRequestQueueSize();
        int numAgents = mindAgents.size();
        if ((0 == reqQueSize) && (0 == numAgents))
            usleep(10000);  // 10 millisecs == 100HZ

        if (reqQueSize != 0) processRequests();
        processMindAgents();

        cycleCount++;
        if (cycleCount < 0) {
            cycleCount = 0;
        }

    } while(true);
}

static int cnt = 0;

void CogServer::processRequests()
{
    int countDown = getRequestQueueSize();
    while (countDown != 0) {
cnt++;
//printf("duude req %d\n", cnt);
if (cnt%23456==0) {
char *h = (char*)sbrk(0);
size_t mem = h - heap_bottom;
mem /= 1024*1024;
printf("cnt=%d at %d MB (%p)\n", cnt, mem, heap_bottom);
}
if (593060 < cnt) printf("now cnt=%d\n", cnt);
        CogServerRequest *request = popRequest();
        request->processRequest();
        delete request;
        countDown--;
    }
}

void CogServer::processMindAgents()
{
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

bool CogServer::getRequestQueueSize() {
    pthread_mutex_lock(&messageQueueLock);
    int size = requestQueue.size();
    pthread_mutex_unlock(&messageQueueLock);
    return size;
}
