/*
 * opencog/learning/PatternMiner/DistributedPatternMiner.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> in 2017
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

#ifndef _OPENCOG_PATTERNMINER_DISTRIBUTEDPATTERNMINER_H
#define _OPENCOG_PATTERNMINER_DISTRIBUTEDPATTERNMINER_H
#include <cstdio>
#include <map>
#include <mutex>
#include <thread>
#include <vector>
#include <cpprest/http_client.h>
#include <cpprest/http_listener.h>

#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atomspace/AtomSpace.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace std;
using namespace web;
using namespace web::http;
using namespace web::http::client;
using namespace web::http::experimental::listener;

namespace opencog
{
namespace PatternMining
{

#define JSON_BUF_MAX_NUM 30

// The implementation of client is in PatternMinerDistributedWorker.cc
// The implementation of server is in PatternMinerCentralServer.cc
class DistributedPatternMiner : public PatternMiner
{

protected:


    string centralServerIP;
    string centralServerPort;
    string centralServerBaseURL;

    web::json::value *patternJsonArrays;

    unsigned int pattern_parse_thread_num; // for the central server
    std::thread centralServerListeningThread;
    std::thread *parsePatternTaskThreads;
    std::mutex patternQueueLock, addRelationLock, updatePatternCountLock, modifyWorkerLock;

    // map<uid, <is_still_working, processedFactsNum>>
    map<string, std::pair<bool, unsigned int>> allWorkers;
    bool allWorkersStop;

    list<json::value> waitForParsePatternQueue;

    string clientWorkerUID;

    bool run_as_distributed_worker;
    bool run_as_central_server;

    http_client* httpClient;
    http_listener* serverListener;

    int cur_worker_mined_pattern_num;
    int total_pattern_received; // in the server

    bool waitingForNewClients;

    // --------------------- overload functions for distributed version --------------------------

    void growPatternsDepthFirstTask(unsigned int thread_index);

    void runPatternMinerDepthFirst();

    // --------------------- end overload functions for distributed version --------------------------

    void handlePost(http_request request);
    void handleRegisterNewWorker(http_request request);
    void handleReportWorkerStop(http_request request);
    void handleFindNewPatterns(http_request request);

    void runParsePatternTaskThread();
    void parseAPatternTask(json::value jval);

    bool checkIfAllWorkersStopWorking();

    void notifyServerThisWorkerStop();

    void startMiningWork();
    void centralServerEvaluateInterestingness();

    void addPatternsToJsonArrayBuf(string curPatternKeyStr, string parentKeyString,  unsigned int extendedLinkIndex, bool notOutPutPattern, json::value &patternJsonArray);
    void sendPatternsToCentralServer(json::value &patternJsonArray);




public:

    DistributedPatternMiner(AtomSpace* _original_as) : PatternMiner(_original_as)
    {

        is_distributed = true;
        patternJsonArrays = new web::json::value[THREAD_NUM];

    }

    void launchADistributedWorker();
    void launchCentralServer();
    void centralServerStartListening();
    void centralServerStopListening();

    bool sendRequest(http_request &request, http_response &response);

    void startCentralServer();


};

}
}

#endif //_OPENCOG_PATTERNMINER_DISTRIBUTEDPATTERNMINER_H
