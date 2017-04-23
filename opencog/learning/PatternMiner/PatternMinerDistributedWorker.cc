/*
 * opencog/learning/PatternMiner/PatternMinerDistributedWorker.cc
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
#include <ifaddrs.h>

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "DistributedPatternMiner.h"

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams


using namespace opencog::PatternMining;
using namespace opencog;


void DistributedPatternMiner::launchADistributedWorker()
{

    run_as_central_server = false;
    run_as_distributed_worker = true;

    centralServerIP = config().get("PMCentralServerIP");
    centralServerPort = config().get("PMCentralServerPort");
    centralServerBaseURL = "http://" +  centralServerIP + ":" + centralServerPort + "/PatternMinerServer";

    boost::uuids::random_generator uuidGen;
    boost::uuids::uuid uid = uuidGen();
    std::stringstream ssuid;
    ssuid << uid;
    clientWorkerUID = ssuid.str();
    std::cout<<"Current client UID = "<< clientWorkerUID << std::endl;
    std::cout<<"Registering to the central server: "<< centralServerIP << std::endl;

    // Create http_client to send the request.
    httpClient = new http_client(U(centralServerBaseURL.c_str()));

    // Build request URI and start the request.
    uri_builder builder(U("/RegisterNewWorker"));
    builder.append_query(U("ClientUID"), U(clientWorkerUID));

    http_request request(methods::POST);
    request.set_request_uri(builder.to_uri());
    http_response response;
    if (sendRequest(request, response))
    {
        // std::cout << response.to_string() << std::endl;
        if (response.status_code() == status_codes::OK)
        {
            std::cout << "Registered to the central server successfully! " << std::endl;
            startMiningWork();
        }
        else
        {
            std::cout << "Registered to the central server failed! Please check network and the the state of the central server.Client application quited!" << std::endl;
            std::exit(EXIT_SUCCESS);
        }
    }
    else
    {
        std::cout << "Network problem. Cannot connect to the central server! Client application quited!" << std::endl;
        std::exit(EXIT_SUCCESS);
    }

}


void DistributedPatternMiner::notifyServerThisWorkerStop()
{


    int tryTimes = 0;
    int maxTryTimes = 10;

    while (tryTimes ++ < maxTryTimes)
    {
        // Build request URI and start the request.
        uri_builder builder(U("/ReportWorkerStop"));
        builder.append_query(U("ClientUID"), U(clientWorkerUID));

        http_request request(methods::POST);
        request.set_request_uri(builder.to_uri());
        http_response response;

        if (sendRequest(request, response))
        {
            std::cout << response.to_string() << std::endl;
            if (response.status_code() == status_codes::OK)
            {
                std::cout << "Report to the central server this worker stopped successfully! " << std::endl;
                break;
            }
            else
            {
                std::cout << "Network problem: Failed report to the central server this worker stopped!" << std::endl;
            }
        }
        else
        {
            std::cout << "Network problem: Failed report to the central server this worker stopped!" << std::endl;
        }

        if (tryTimes < maxTryTimes)
        {
            std::cout << "Wait for 5 seconds and retry..."  << std::endl;
            sleep(5);
        }
        else
        {
            std::cout << "Already tried " << maxTryTimes << " times. Enter 'y' to keep trying, enter others to quit."  << std::endl;

            string inputstr;
            cin  >> inputstr;

            if ( (inputstr == "y") or  (inputstr == "yes") )
            {
                maxTryTimes += 10;
                std::cout << "Continue trying..."  << std::endl;
                sleep(5);
            }
            else
            {
                break;
            }

        }

    }


    cout << "Client quited!" << std::endl;
    std::exit(EXIT_SUCCESS);

}

void DistributedPatternMiner::startMiningWork()
{
    // Note: Breadth_First mining is not maintained anymore. Only Depth_First is used in distributed mining.
    //    Pattern_mining_mode = config().get("Pattern_mining_mode"); // option: Breadth_First , Depth_First
    //    assert( (Pattern_mining_mode == "Breadth_First") || (Pattern_mining_mode == "Depth_First"));
        Pattern_mining_mode = "Depth_First";

        cur_worker_mined_pattern_num = 0;

        std::cout << "Start pattern mining work! Max gram = "
                  << this->MAX_GRAM << ", mode = Depth_First" << std::endl;

        int start_time = time(NULL);

        originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

        allLinkNumber = (int)(allLinks.size());
        atomspaceSizeFloat = (float)(allLinkNumber);

        runPatternMinerDepthFirst();

        int end_time = time(NULL);
        printf("Current pattern mining worker finished working! Total time: %d seconds. \n", end_time - start_time);

        notifyServerThisWorkerStop();

        std::cout << THREAD_NUM << " threads used. "
                  <<"Corpus size: "<< allLinkNumber << " links in total. \n"
                  << cur_worker_mined_pattern_num << " patterns mined in total." << std::endl;

        // std::exit(EXIT_SUCCESS);

}

void DistributedPatternMiner::runPatternMinerDepthFirst()
{
    // observingAtomSpace is used to copy one link everytime from the originalAtomSpace
    observingAtomSpace = new AtomSpace();

//    cur_DF_ExtractedLinks = new set<string>[MAX_GRAM];

    linksPerThread = allLinkNumber / THREAD_NUM;

    processedLinkNum = 0;
    actualProcessedLinkNum = 0;

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {

        threads[i] = std::thread(&DistributedPatternMiner::growPatternsDepthFirstTask,this,i);
        // threads[i] = std::thread([this]{this->growPatternsDepthFirstTask(i);}); // using C++11 lambda-expression
    }

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i].join();
    }

    // release allLinks
    allLinks.clear();
    (HandleSeq()).swap(allLinks);

//    delete [] cur_DF_ExtractedLinks;
    delete [] threads;

    cout << "\nFinished mining 1~" << MAX_GRAM << " gram patterns.\n";
    cout << "\nprocessedLinkNum = " << processedLinkNum << std::endl;

    delete [] patternJsonArrays;
    cout << "Totally "<< cur_worker_mined_pattern_num << " patterns found!\n";
}

void DistributedPatternMiner::growPatternsDepthFirstTask(unsigned int thread_index)
{

    // The start index in allLinks for current thread
    unsigned int start_index = linksPerThread * thread_index;
    unsigned int end_index; // the last index for current thread (excluded)
    if (thread_index == THREAD_NUM - 1) // if this the last thread, it
                                        // needs to finish all the
                                        // rest of the links
        end_index = allLinkNumber;
    else
        end_index = linksPerThread * (thread_index + 1);


    cout << "Start thread " << thread_index << ": will process Link number from " << start_index
         << " to (excluded) " << end_index << std::endl;

    patternJsonArrays[thread_index] = json::value::array();

    float allLinkNumberfloat = ((float)(end_index - start_index));
    for(unsigned int t_cur_index = start_index; t_cur_index < end_index; ++t_cur_index)
    {
        readNextLinkLock.lock();
        cout<< "\r" << ((float)(t_cur_index - start_index))/allLinkNumberfloat*100.0f << "% completed in Thread " + toString(thread_index) + "."; // it's not liner
        std::cout.flush();

        processedLinkNum ++;
        Handle& cur_link = allLinks[t_cur_index];

        readNextLinkLock.unlock();

        // if this link is ingonre type, ignore it
        if (isIgnoredType( cur_link->getType()))
        {
            continue;
        }


        if (use_keyword_black_list)
        {
            // if the content in this link contains content in the black list,ignore it
            if (containIgnoredContent(cur_link))
                continue;
        }

        // Add this link into observingAtomSpace
        HandleSeq outgoingLinks, outVariableNodes;

        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
        Handle newLink = observingAtomSpace->add_link(cur_link->getType(), outgoingLinks);
        newLink->merge(cur_link->getTruthValue());


        // Extract all the possible patterns from this originalLink, and extend till the max_gram links, not duplicating the already existing patterns
        HandleSeq lastGramLinks;
        map<Handle,Handle> lastGramValueToVarMap;
        map<Handle,Handle> patternVarMap;

        set<string> allNewMinedPatternsCurTask;


        // allHTreeNodesCurTask is only used in distributed version;
        // is to store all the HTreeNode* mined in this current task, and release them after the task is finished.
        vector<HTreeNode*> allHTreeNodesCurTask;

        // allNewMinedPatternInfo is only used in distributed version, to store all the new mined patterns in this task for sending to the server.
        vector<MinedPatternInfo> allNewMinedPatternInfo;


        actualProcessedLinkLock.lock();
        actualProcessedLinkNum ++;
        actualProcessedLinkLock.unlock();

        extendAPatternForOneMoreGramRecursively(newLink, observingAtomSpace, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,
                                                patternVarMap, false, allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo);


        // send new mined patterns to the server
        for (MinedPatternInfo& pInfo : allNewMinedPatternInfo)
            addPatternsToJsonArrayBuf(pInfo.curPatternKeyStr, pInfo.parentKeyString, pInfo.extendedLinkIndex, patternJsonArrays[thread_index]);

        // release all the HTreeNodes created in this task
        // clean up the pattern atomspace, do not need to keep patterns in atomspace when run as a distributed worker
        if (THREAD_NUM == 1)
            atomSpace->clear(); // can only clear the atomspace when only 1 thread is used

        for(unsigned int hNodeNum = 0; hNodeNum < allHTreeNodesCurTask.size(); hNodeNum ++)
        {
            delete (allHTreeNodesCurTask[hNodeNum]);
        }


    }

    if (patternJsonArrays[thread_index].size() > 0)
        sendPatternsToCentralServer(patternJsonArrays[thread_index]);

    cout<< "\r100% completed in Thread " + toString(thread_index) + ".";
    std::cout.flush();
}

void DistributedPatternMiner::addPatternsToJsonArrayBuf(string curPatternKeyStr, string parentKeyString,  unsigned int extendedLinkIndex, json::value &patternJsonArray)
{

    try
    {

        json::value patternInfo = json::value::object();
        patternInfo[U("Pattern")] = json::value(U(curPatternKeyStr));
        patternInfo[U("ParentPattern")] = json::value(U(parentKeyString));
        patternInfo[U("ExtendedLinkIndex")] = json::value(U(extendedLinkIndex));
        patternInfo[U("ClientUID")] = json::value(U(clientWorkerUID));
        patternInfo[U("ProcessedFactsNum")] = json::value(U(actualProcessedLinkNum));

        patternJsonArray[patternJsonArray.size()] = patternInfo;

        if(patternJsonArray.size() >= JSON_BUF_MAX_NUM)
            sendPatternsToCentralServer(patternJsonArray);

    }
    catch (exception const & e)
    {
       cout << e.what() << "addPatternsToJsonArrayBuf exception!: " << endl;

    }

    cur_worker_mined_pattern_num ++;
    // cout << "\nWorker added cur_worker_mined_pattern_num = " << cur_worker_mined_pattern_num << std::endl;
}

// this function will empty patternJsonArray after sent
void DistributedPatternMiner::sendPatternsToCentralServer(json::value &patternJsonArray)
{

    uri_builder builder(U("/FindNewPatterns"));

    while (true)
    {
        http_request request(methods::POST);
        request.set_request_uri(builder.to_uri());
        request.headers().add(header_names::accept, U("application/json"));
        request.set_body(patternJsonArray);

        http_response response;

        if (sendRequest(request, response))
        {
//            std::cout << response.to_string() << std::endl;
            if (response.status_code() == status_codes::OK)
            {
                break;
            }
            else
            {
                std::cout << "Network problem: Failed to send new found patterns! Retrying ... " << std::endl;
            }
        }
        else
        {
            std::cout << "Network problem: Failed to send new found patterns! Retrying ... " << std::endl;
        }

    }


    // empty the array
    patternJsonArray = json::value::array();

    usleep(20000);


}

// send request, get response back
bool DistributedPatternMiner::sendRequest(http_request &request, http_response &response)
{
    pplx::task<http_response> task = httpClient->request(request);

    try
    {
        if (task.wait() == pplx::task_status::completed)
        {
            response = task.get();
//            response.StatusCode = task.get().status_code();
//            pplx::task<web::json::value> contentTask = task.get().extract_json();
//            if (contentTask.wait() == task_status::completed)
//            {
//                response.JsonContent = contentTask.get();
//            }

            return true;
        }
        else
        {
            cout << "Network problem:http request task is not finished! " << std::endl;
            return false;
        }
    }
    catch (http_exception e)
    {
        cout << "http_exception:" << e.what() << std::endl;
        return false;
    }
    catch (exception e)
    {
        cout << "exception:" << e.what() << std::endl;
        return false;
    }



}

