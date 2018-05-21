/*
 * opencog/learning/PatternMiner/PatternMinerCentralServer.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> in 2015
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

#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <sstream>
#include <thread>
#include <set>
#include <string>
#include <functional>
#include <sys/time.h>

#include <boost/range/algorithm/sort.hpp>

#include <opencog/atoms/base/Handle.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>

#include "DistributedPatternMiner.h"

#include <cpprest/http_listener.h>
#include <cpprest/http_msg.h>
#include <cpprest/json.h>


using namespace opencog::PatternMining;
using namespace opencog;

using namespace std;
using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;
using namespace utility;


bool compareHTreeNodeByFrequency(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness_I(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness_II(HTreeNode* node1, HTreeNode* node2);


void DistributedPatternMiner::launchCentralServer()
{
    run_as_distributed_worker = false;
    run_as_central_server = true;

    waitingForNewClients = false;

    centralServerPort = config().get("PMCentralServerPort");

    centralServerIP = config().get("PMCentralServerIP");

    pattern_parse_thread_num = (unsigned int)(config().get_int("pattern_parse_thread_num"));

    serverListener = new http_listener( utility::string_t("http://" + centralServerIP + ":" + centralServerPort +"/PatternMinerServer") );

    serverListener->support(methods::POST, std::bind(&DistributedPatternMiner::handlePost, this,  std::placeholders::_1));

    allWorkersStop = false;

    centralServerListeningThread = std::thread([this]{this->centralServerStartListening();});


    while (true)
    {
        parsePatternTaskThreads = new thread[pattern_parse_thread_num];

        for (unsigned int i = 0; i < pattern_parse_thread_num; ++ i)
        {
            parsePatternTaskThreads[i] = std::thread([this]{this->runParsePatternTaskThread();});

        }

        cout <<"\nPattern Miner central server started! "<< pattern_parse_thread_num << " threads using to parse patterns." << std::endl;

        for (unsigned int i = 0; i < pattern_parse_thread_num; ++ i)
        {
            parsePatternTaskThreads[i].join();

        }

        delete [] parsePatternTaskThreads;

        string inputstr;
        cout << "All connected clients have finished and all the received patterns have been parsed by the server.\n"
             << "Enter 'y' or 'yes' to start evaluating pattern interestingness.\n"
             << "Enter any other words to keep waiting for more clients to connect" << std::endl;
        cin  >> inputstr;

        if ( (inputstr == "y") or  (inputstr == "yes") )
        {
            // calculate the corpus size by adding up all the processedFactsNum of every worker
            unsigned int totalProcessedFactsNum = 0;
            for (const auto& worker : allWorkers)
                totalProcessedFactsNum += worker.second.second;

            atomspaceSizeFloat = (float) totalProcessedFactsNum;

            cout <<"\n Pattern mining finished in the central server! \n"
                 << "Totally " << totalProcessedFactsNum << " facts processed! "
                 << keyStrToHTreeNodeMap.size() << " pattern found!\n"
                 << "Now start to evaluate interestingness." << std::endl;

            centralServerEvaluateInterestingness();

            break;
        }
        else
        {
            waitingForNewClients = true;
            cout <<"Wainting for new clients to connect..." << std::endl;

        }
    }



}

void DistributedPatternMiner::centralServerStartListening()
{
    try
    {
       total_pattern_received = 0;
       serverListener->open().wait();

    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
    }
}

void DistributedPatternMiner::centralServerStopListening()
{
    try
    {
       cout << "\nCentral Server Stop Listening! total_pattern_received = " << total_pattern_received << std::endl;
       serverListener->close().wait();

    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
    }
}

bool DistributedPatternMiner::checkIfAllWorkersStopWorking()
{

    if (allWorkers.empty())
        return false; // has not started yet

    for (const auto& worker : allWorkers)
        if (worker.second.first)
            return false;

    return true;
}

void DistributedPatternMiner::handlePost(http_request request)
{
    try
    {
        // cout << "Got request: \n" << request.to_string() << std::endl;

        string path = request.relative_uri().path();
        if (path == "/RegisterNewWorker")
        {
            handleRegisterNewWorker(request);
        }
        else if (path == "/FindNewPatterns")
        {
            handleFindNewPatterns(request);
        }
        else if (path == "/ReportWorkerStop")
        {
            handleReportWorkerStop(request);
        }
        else
        {
            json::value answer = json::value::object();

            answer["msg"] = json::value("Unknown request command!");

            request.reply(status_codes::NotFound, answer);
        }
    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
       request.reply(status_codes::NotFound);
    }
}

void DistributedPatternMiner::handleRegisterNewWorker(http_request request)
{
    try
    {
        auto paths = uri::split_path(uri::decode(request.relative_uri().query()));

        string clientID =  paths[0];
        // cout << "\nGot request to RegisterNewWorker: ClientID = \n" << clientID << std::endl;

        modifyWorkerLock.lock();
        allWorkersStop = false;
        waitingForNewClients = false;
        allWorkers.insert({clientID, std::pair<bool, unsigned int>(true, 0)}); // true means this worker is still working.
        modifyWorkerLock.unlock();

        json::value answer = json::value::object();
        answer["msg"] = json::value("Worker registered successfully!");
        request.reply(status_codes::OK, answer);
        cout << "\nA new worker connected! ClientID = \n" << clientID << std::endl;

    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
       request.reply(status_codes::NotFound);
    }



}

void DistributedPatternMiner::handleReportWorkerStop(http_request request)
{
    try
    {
        auto paths = uri::split_path(uri::decode(request.relative_uri().query()));

        string clientID =  paths[0];
        cout << "Got request to ReportWorkerStop: ClientID = \n" << clientID << std::endl;

        modifyWorkerLock.lock();

        map<string, std::pair<bool, unsigned int>>::iterator wokerIt = allWorkers.find(clientID);
        if (wokerIt == allWorkers.end())
            cout << "ClientID " << clientID <<  " not found!" <<std::endl;
        else
        {
            (wokerIt->second).first = false;
            allWorkersStop = checkIfAllWorkersStopWorking();
            if (allWorkersStop)
                cout << "All workers connected to this server have stopped!" <<std::endl;
        }

        modifyWorkerLock.unlock();


        request.reply(status_codes::OK);
    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
       request.reply(status_codes::NotFound);
    }

}



//long startTime;
//long endTime;
void DistributedPatternMiner::handleFindNewPatterns(http_request request)
{

//    // check server load
//    static int msgReceivedNum = 0;
//    msgReceivedNum ++;

//    if (msgReceivedNum == 100)
//    {
//        timeval t1;
//        gettimeofday(&t1, nullptr);

//        startTime = t1.tv_sec*1000 + t1.tv_usec/1000;
//    }

//    if (msgReceivedNum == 2100)
//    {
//        timeval t2;
//        gettimeofday(&t2, nullptr);

//        endTime = t2.tv_sec*1000 + t2.tv_usec/1000;
//        long costSeconds = (endTime - startTime) / 1000;
//        std::cout<<std::endl<<"Server recieved 2000 requests in " << costSeconds << " seconds" << std::endl;
//        std::cout<< (int)(2000.0f / (float)costSeconds) << " requests per second in average! " << std::endl;
//    }

    request.reply(status_codes::OK);

    patternQueueLock.lock();

    try
    {
        // should get an array of patterns
        json::value jarray = request.extract_json().get();

        for (const json::value& jval : jarray.as_array())
        {
            waitForParsePatternQueue.push_back(jval);
            total_pattern_received ++;

            //  cout << "Received pattern : \n" << jval.serialize() << std::endl;
        }

        // cout << "Received pattern num = " << total_pattern_received << std::endl;


    }
    catch (exception const & e)
    {
       cout << e.what() << "handleFindANewPattern error: " << request.to_string() << endl;

    }

    patternQueueLock.unlock();
}

void DistributedPatternMiner::runParsePatternTaskThread()
{
    static int tryToParsePatternNum = 0;

    while(true)
    {

        patternQueueLock.lock();
        if (not waitForParsePatternQueue.empty())
        {
            json::value jval = waitForParsePatternQueue.front();
            waitForParsePatternQueue.pop_front();

            tryToParsePatternNum ++;
            cout<< "\r" << tryToParsePatternNum << + " received patterns parsed." ;
            std::cout.flush();

            patternQueueLock.unlock();

            parseAPatternTask(jval);

        }
        else
        {
            patternQueueLock.unlock();

            if (allWorkersStop && (! waitingForNewClients))
                return;
        }


    }
}

void DistributedPatternMiner::parseAPatternTask(json::value jval)
{

    try
    {
        string PatternStr =  jval[U("Pattern")].as_string();
        // cout << "PatternStr = " << PatternStr << std::endl;
        string ParentPatternStr = jval[U("ParentPattern")].as_string();
        // cout << "ParentPatternStr = " << ParentPatternStr << std::endl;
        int ExtendedLinkIndex = jval[U("ExtendedLinkIndex")].as_integer();
        bool notOutPutPattern = jval[U("notOutPutPattern")].as_bool();
        // cout << "ExtendedLinkIndex = " << ExtendedLinkIndex << std::endl;
        string clientIDStr = jval[U("ClientUID")].as_string();
        unsigned int processedFactsNum = (unsigned int)(jval[U("ProcessedFactsNum")].as_integer());

        modifyWorkerLock.lock();
        allWorkers[clientIDStr].second = processedFactsNum;
        modifyWorkerLock.unlock();

        HTreeNode* newHTreeNode;
        // check if the pattern key already exist
        uniqueKeyLock.lock();
        map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(PatternStr);
        uniqueKeyLock.unlock();

        static int duplicatePatternNum = 0;

        if (htreeNodeIter != keyStrToHTreeNodeMap.end())
        {
            newHTreeNode = htreeNodeIter->second;

            updatePatternCountLock.lock();
            newHTreeNode->count ++;
            duplicatePatternNum ++;
            // cout << "\nduplicatePatternNum = " << duplicatePatternNum << std::endl;
            updatePatternCountLock.unlock();
        }
        else
        {
            // add this new found pattern into the Atomspace
            HandleSeq patternHandleSeq = loadPatternIntoAtomSpaceFromString(PatternStr, *as);

            if (patternHandleSeq.empty())
            {

                // cout << "Warning: Invalid pattern string: " << PatternStr << std::endl;
                return;

            }

            // create a new HTreeNode
            newHTreeNode = new HTreeNode();
            newHTreeNode->pattern = patternHandleSeq;
            newHTreeNode->count = 1;
            uniqueKeyLock.lock();
            keyStrToHTreeNodeMap.insert({PatternStr, newHTreeNode});
            uniqueKeyLock.unlock();

        }

        if (newHTreeNode->count == 1)
        {
            addNewPatternLock.lock();

            if (notOutPutPattern)
                (tmpPatternsForGram[newHTreeNode->pattern.size()-1]).push_back(newHTreeNode);
            else
                (patternsForGram[newHTreeNode->pattern.size()-1]).push_back(newHTreeNode);

            addNewPatternLock.unlock();
        }

        if (newHTreeNode->pattern.size() > 1) // this pattern is more than 1 gram, it should have a parent pattern
        {
            if (ParentPatternStr == "none")
            {

                // cout << "Warning: Pattern missing parent pattern string:" << PatternStr;
                return;

            }
            else
            {
                uniqueKeyLock.lock();
                map<string, HTreeNode*>::iterator parentNodeIter = keyStrToHTreeNodeMap.find(ParentPatternStr);
                uniqueKeyLock.unlock();

                HTreeNode* parentNode;

                if (parentNodeIter == keyStrToHTreeNodeMap.end())
                {

                    // cout << "Warning: Cannot find parent pattern in server: " << ParentPatternStr << std::endl;
                    // The parent pattern should have been added, but it is not.
                    // It's possibly sent by another thread, and added to the queue later than the current pattern.
                    // So we add this parent pattern here first, but set its count = 0
                    // add this new found pattern into the Atomspace
                    HandleSeq parentPatternHandleSeq = loadPatternIntoAtomSpaceFromString(ParentPatternStr, *as);
                    if (parentPatternHandleSeq.empty())
                    {

                        cout << "Warning: Invalid pattern string: " << ParentPatternStr << std::endl;
                        return;

                    }

                    // create a new HTreeNode
                    parentNode = new HTreeNode();
                    parentNode->pattern = parentPatternHandleSeq;
                    parentNode->count = 0;

                    uniqueKeyLock.lock();
                    keyStrToHTreeNodeMap.insert({ParentPatternStr, parentNode});
                    uniqueKeyLock.unlock();

//                    addNewPatternLock.lock();
//                    (patternsForGram[parentPatternHandleSeq.size()-1]).push_back(parentNode);
//                    addNewPatternLock.unlock();

                }
                else
                    parentNode = parentNodeIter->second;


                // add super pattern relation



                ExtendRelation relation;
                relation.extendedHTreeNode = newHTreeNode;
                relation.newExtendedLink = (newHTreeNode->pattern)[ExtendedLinkIndex];

                addRelationLock.lock();
                parentNode->superPatternRelations.push_back(relation);
                addRelationLock.unlock();

            }
        }
    }
    catch (exception const & e)
    {
       cout << e.what() << endl;
    }
}



void DistributedPatternMiner::centralServerEvaluateInterestingness()
{

    std::cout<<"Debug: PatternMiner:  done frequent pattern mining for 1 to "<< param.MAX_GRAM <<"gram patterns!\n";

    for (unsigned int gram = 1; gram <= param.MAX_GRAM; gram++)
    {
        // sort by frequency
        boost::sort(patternsForGram[gram-1], compareHTreeNodeByFrequency );

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

        OutPutFrequentPatternsToFile(gram, patternsForGram);

        std::cout<< std::endl;
    }



    if (param.enable_interesting_pattern)
    {
        for (cur_gram = 2; cur_gram <= param.MAX_GRAM; cur_gram++)
        {
            cout << "\nCalculating";
            if (param.enable_interaction_information)
                cout << " Interaction_Information ";

            if (param.enable_surprisingness)
                cout << " Surprisingness ";

            cout << "for " << cur_gram << " gram patterns." << std::endl;

            cur_index = -1;
            threads = new thread[param.THREAD_NUM];
            num_of_patterns_without_superpattern_cur_gram = 0;

            for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
            {
                threads[i] = std::thread([this]{this->evaluateInterestingnessTask();}); // using C++11 lambda-expression
            }

            for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
            {
                threads[i].join();
            }

            delete [] threads;

            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interestingness evaluation!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! ";
            std::cout<<"Outputting to file ... ";

            if (param.enable_interaction_information)
            {
                // sort by interaction information
                boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeByInteractionInformation);
                OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 0);
            }

            if (param.enable_surprisingness)
            {
                // sort by surprisingness_I first
                boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeBySurprisingness_I);
                OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);

                if (cur_gram == param.MAX_GRAM)
                    break;

                vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];

                // and then sort by surprisingness_II
                boost::sort(curGramPatterns, compareHTreeNodeBySurprisingness_II);
                OutPutInterestingPatternsToFile(curGramPatterns,cur_gram,2);

                // Get the min threshold of surprisingness_II
                int threshold_index_II;
                threshold_index_II = SURPRISINGNESS_II_TOP_THRESHOLD * (float)(curGramPatterns.size() - num_of_patterns_without_superpattern_cur_gram);
                int looptimes = 0;
                while (true)
                {

                    surprisingness_II_threshold = (curGramPatterns[threshold_index_II])->nII_Surprisingness;
                    if (surprisingness_II_threshold <= 0.00000f)
                    {
                        if (++ looptimes > 8)
                        {
                            surprisingness_II_threshold = 0.00000f;
                            break;
                        }

                        threshold_index_II = ((float)threshold_index_II) * SURPRISINGNESS_II_TOP_THRESHOLD;
                    }
                    else
                        break;
                }

                cout<< "surprisingness_II_threshold for " << cur_gram << " gram = "<< surprisingness_II_threshold;

                // go through the top N patterns of surprisingness_I, pick the patterns with surprisingness_II higher than threshold
                int threshold_index_I = SURPRISINGNESS_I_TOP_THRESHOLD * (float)(curGramPatterns.size());
                for (int p = 0; p <= threshold_index_I; p ++)
                {
                    HTreeNode* pNode = (patternsForGram[cur_gram-1])[p];

                    // for patterns have no superpatterns, nII_Surprisingness == -1.0, which should be taken into account
                    if ( (pNode->nII_Surprisingness < 0 ) || (pNode->nII_Surprisingness > surprisingness_II_threshold ) )
                        finalPatternsForGram[cur_gram-1].push_back(pNode);
                }

                // sort by frequency
                boost::sort(finalPatternsForGram[cur_gram-1], compareHTreeNodeByFrequency );

                OutPutFinalPatternsToFile(cur_gram);
            }
            std::cout<< std::endl;
        }
    }

    std::cout << "Pattern Miner application quited!" << std::endl;
    std::exit(EXIT_SUCCESS);
}
