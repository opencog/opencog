/**
 * CogServer.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Jun 20 16:00:19 BRT 2007
 */

#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "CogServer.h"
#include "exceptions.h"
#include "Config.h"
#include "Logger.h"
#include "SimpleNetworkServer.h"

using namespace opencog;
AtomSpace* CogServer::atomSpace = NULL;

CogServer::~CogServer() {
    delete networkServer;
}

CogServer::CogServer() : cycleCount(1), networkServer(NULL) {
    pthread_mutex_init(&messageQueueLock, NULL);
}

void CogServer::init() {
    if (atomSpace != NULL) {
        delete atomSpace;
    }
    atomSpace = new AtomSpace();

    // setup main logger
    //Util::Logger *log = new Util::Logger(config()["LOG_FILE"], Util::Logger::INFO, true);
    logger().setPrintToStdoutFlag(true);

    // set network server
    this->networkServer =
        new SimpleNetworkServer(this, config().get_int("SERVER_PORT"));
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

void CogServer::serverLoop() {
    struct timeval timer_start, timer_end;
    time_t elapsed_time;
    time_t cycle_duration = config().get_int("SERVER_CYCLE_DURATION") * 1000;

    if (networkServer != NULL) networkServer->start();

    logger().info("opencog server ready.");
    running = true;
    while (running) {
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

// create and return the single instance
CogServer& opencog::server() {
    static CogServer instance;
    return instance;
}
