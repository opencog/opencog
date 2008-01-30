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

using namespace opencog;

AtomSpace *CogServer::atomSpace = NULL;

unsigned sleep(unsigned seconds);

CogServer::~CogServer() {
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

void CogServer::serverLoop() {

    initializer->setUp(this);
    if (networkServer != NULL) {
        networkServer->start();
    }

    do {
        processRequests();
        processMindAgents();

        cycleCount++;
        if (cycleCount < 0) {
            cycleCount = 0;
        }

    } while(true);
}

void CogServer::processRequests() {

    int countDown = getRequestQueueSize();
    while (countDown != 0) {
        CogServerRequest *request = popRequest();
        if (requestProcessor.find(request->getType()) == requestProcessor.end()) {
           throw new RuntimeException(NULL, "Unable to find request processor for <%s>", request->getType().c_str());
        }
        requestProcessor[request->getType()]->processRequest(request);
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

void CogServer::plugInRequestProcessor(const std::string &requestType, RequestProcessor *processor) {
    requestProcessor[requestType] = processor;
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
